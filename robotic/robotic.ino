#include <DynamixelShield.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

using namespace ControlTableItem;

// --------------------------------------------------
// 几何参数：机械臂连杆长度
// --------------------------------------------------
const float L1 = 10.0f;
const float L2 = 10.0f;

// --------------------------------------------------
// 舵机ID
// --------------------------------------------------
const uint8_t ID_6 = 6;
const uint8_t ID_8 = 8;
const uint8_t ID_2 = 2;
const uint8_t ID_1 = 1;

// --------------------------------------------------
// 初始偏置
// --------------------------------------------------
const float BIAS_6 = 150.0f;
const float BIAS_8 = 60.0f;
const float BIAS_2 = 60.0f;
const float BIAS_1 = 60.0f;

// --------------------------------------------------
// 连续性状态
// --------------------------------------------------
static bool g_hasPreviousTheta = false;
static float g_previousThetaDeg = 0.0f;

static bool g_hasPreviousIKSolution = false;
static float g_previousAlphaDeg = 0.0f;
static float g_previousBetaDeg = 0.0f;

// --------------------------------------------------
// 角度工具函数
// --------------------------------------------------
float wrapAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg <= -180.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

float unwrapAngleToReferenceDeg(float angle_deg, float reference_deg)
{
  float result = angle_deg;
  while (result - reference_deg > 180.0f)
  {
    result -= 360.0f;
  }
  while (result - reference_deg < -180.0f)
  {
    result += 360.0f;
  }
  return result;
}

float shortestAngleDiffDeg(float a_deg, float b_deg)
{
  return fabs(wrapAngleDeg(a_deg - b_deg));
}

// --------------------------------------------------
// 只在初始化时：把舵机切到位置模式
// --------------------------------------------------
void initServoPositionMode(uint8_t id)
{
  dxl.torqueOff(id);
  delay(30);

  dxl.setOperatingMode(id, OP_POSITION);
  delay(30);

  dxl.torqueOn(id);
  delay(30);
}

// --------------------------------------------------
// 只负责写目标角度，不再切模式
// --------------------------------------------------
void setServoPosition(uint8_t id, float target_deg)
{
  dxl.setGoalPosition(id, target_deg, UNIT_DEGREE);
}

// --------------------------------------------------
// 分别设置4个舵机的角度
// --------------------------------------------------
void set_position6(float deg)
{
  setServoPosition(ID_6, deg + BIAS_6);
}

void set_position8(float deg)
{
  setServoPosition(ID_8, deg + BIAS_8);
}

void set_position2(float deg)
{
  setServoPosition(ID_2, deg + BIAS_2);
}

void set_position1(float deg)
{
  setServoPosition(ID_1, deg + BIAS_1);
}

// --------------------------------------------------
// 逆运动学：由目标 x, y 求 alpha, beta
// 双解时，选择与上一帧更接近的一支
// --------------------------------------------------
void inverseKinematics(float x, float y, float &alpha_deg, float &beta_deg)
{
  float r = sqrt(x * x + y * y);

  // 这里不做 r == 0 保护，按你的要求由 MATLAB 侧保证
  if (r > (L1 + L2) || r < fabs(L1 - L2))
  {
    alpha_deg = NAN;
    beta_deg = NAN;
    return;
  }

  float cosBeta = (L1 * L1 + L2 * L2 - r * r) / (2.0f * L1 * L2);
  if (cosBeta > 1.0f) cosBeta = 1.0f;
  if (cosBeta < -1.0f) cosBeta = -1.0f;

  float beta = acos(cosBeta);
  float betaCandidate = beta * 180.0f / PI;

  float phi = atan2(y, x);

  float cosGamma = (L1 * L1 + r * r - L2 * L2) / (2.0f * L1 * r);
  if (cosGamma > 1.0f) cosGamma = 1.0f;
  if (cosGamma < -1.0f) cosGamma = -1.0f;

  float gamma = acos(cosGamma);

  // 两个候选分支
  float alphaCandidate1 = (PI - (phi + gamma)) * 180.0f / PI;
  float alphaCandidate2 = (PI - (phi - gamma)) * 180.0f / PI;

  // 第一次没有上一帧，默认取第一个分支
  if (!g_hasPreviousIKSolution)
  {
    alpha_deg = alphaCandidate1;
    beta_deg = betaCandidate;
  }
  else
  {
    // 先把两个候选角都展开到和上一帧接近的附近
    float alpha1Unwrapped = unwrapAngleToReferenceDeg(alphaCandidate1, g_previousAlphaDeg);
    float alpha2Unwrapped = unwrapAngleToReferenceDeg(alphaCandidate2, g_previousAlphaDeg);

    float betaUnwrapped = unwrapAngleToReferenceDeg(betaCandidate, g_previousBetaDeg);

    float score1 = shortestAngleDiffDeg(alpha1Unwrapped, g_previousAlphaDeg) +
                   shortestAngleDiffDeg(betaUnwrapped, g_previousBetaDeg);

    float score2 = shortestAngleDiffDeg(alpha2Unwrapped, g_previousAlphaDeg) +
                   shortestAngleDiffDeg(betaUnwrapped, g_previousBetaDeg);

    if (score2 < score1)
    {
      alpha_deg = alpha2Unwrapped;
    }
    else
    {
      alpha_deg = alpha1Unwrapped;
    }

    beta_deg = betaUnwrapped;
  }

  g_previousAlphaDeg = alpha_deg;
  g_previousBetaDeg = beta_deg;
  g_hasPreviousIKSolution = true;
}

// --------------------------------------------------
// 由目标坐标 (x, y) 直接驱动 8 号和 2 号舵机
// --------------------------------------------------
void moveToXY(float x, float y)
{
  float alpha_deg = NAN;
  float beta_deg = NAN;

  inverseKinematics(x, y, alpha_deg, beta_deg);

  if (isnan(alpha_deg) || isnan(beta_deg))
  {
    return;
  }

  set_position8(alpha_deg);
  set_position2(beta_deg);
}

// --------------------------------------------------
// 由三维坐标 (X, Y, Z) 控制机械臂
// --------------------------------------------------
void moveToXYZ(float X, float Y, float Z, float error_deg)
{
  float rho = sqrt(X * X + Y * Y);
  float theta_deg = atan2(Y, X) * 180.0f / PI;

  // 6号舵机方位角做连续展开
  if (g_hasPreviousTheta)
  {
    theta_deg = unwrapAngleToReferenceDeg(theta_deg, g_previousThetaDeg);
  }
  g_previousThetaDeg = theta_deg;
  g_hasPreviousTheta = true;

  moveToXY(rho, Z);
  set_position6(theta_deg);

  float alpha_deg = NAN;
  float beta_deg = NAN;
  inverseKinematics(rho, Z, alpha_deg, beta_deg);

  if (isnan(alpha_deg) || isnan(beta_deg))
  {
    return;
  }

  float gamma_deg = 90.0f + (beta_deg - alpha_deg) + error_deg;
  set_position1(gamma_deg);
}

// --------------------------------------------------
// 读取一帧串口命令：#X,Y,Z,error$
// 例如：#+05.0,-06.0,+07.0,+30.0$
// 返回 true 表示成功解析到一组新数据
// --------------------------------------------------
bool readXYZErrorFromSerial(float &X, float &Y, float &Z, float &error_deg)
{
  static char buffer[80];
  static uint8_t idx = 0;
  static bool inFrame = false;

  while (DEBUG_SERIAL.available() > 0)
  {
    char c = (char)DEBUG_SERIAL.read();

    // 帧头：开始接收
    if (c == '#')
    {
      idx = 0;
      inFrame = true;
      continue;
    }

    // 还没进入帧内，忽略所有字符
    if (!inFrame)
    {
      continue;
    }

    // 忽略回车换行
    if (c == '\r' || c == '\n')
    {
      continue;
    }

    // 帧尾：结束并解析
    if (c == '$')
    {
      buffer[idx] = '\0';
      idx = 0;
      inFrame = false;

      char *token = strtok(buffer, ",");
      if (!token) return false;
      X = atof(token);

      token = strtok(NULL, ",");
      if (!token) return false;
      Y = atof(token);

      token = strtok(NULL, ",");
      if (!token) return false;
      Z = atof(token);

      token = strtok(NULL, ",");
      if (!token) return false;
      error_deg = atof(token);

      return true;
    }

    // 正常字符写入缓冲区
    if (idx < sizeof(buffer) - 1)
    {
      buffer[idx++] = c;
    }
    else
    {
      // 缓冲区溢出：丢弃本帧
      idx = 0;
      inFrame = false;
    }
  }

  return false;
}

void moveBetweenXYZ(float x1, float y1, float z1, float error1,
                    float x2, float y2, float z2, float error2)
{
  const float STEP = 0.2f;
  const unsigned long INTERVAL_MS = 50;

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float de = error2 - error1;

  float maxDelta = fabs(dx);
  if (fabs(dy) > maxDelta) maxDelta = fabs(dy);
  if (fabs(dz) > maxDelta) maxDelta = fabs(dz);
  if (fabs(de) > maxDelta) maxDelta = fabs(de);

  int steps = (int)ceil(maxDelta / STEP);
  if (steps < 1) steps = 1;

  for (int i = 0; i <= steps; i++)
  {
    float t = (float)i / (float)steps;

    float x = x1 + t * dx;
    float y = y1 + t * dy;
    float z = z1 + t * dz;
    float error_deg = error1 + t * de;

    moveToXYZ(x, y, z, error_deg);
    delay(INTERVAL_MS);
  }
}

void setup()
{
  DEBUG_SERIAL.begin(38400);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ledOff(ID_6);
  dxl.ledOff(ID_8);
  dxl.ledOff(ID_2);
  dxl.ledOff(ID_1);

  delay(100);

  initServoPositionMode(ID_6);
  initServoPositionMode(ID_8);
  initServoPositionMode(ID_2);
  initServoPositionMode(ID_1);
}

void loop()
{
  float X, Y, Z, error_deg;

  if (readXYZErrorFromSerial(X, Y, Z, error_deg))
  {
    moveToXYZ(X, Y, Z, error_deg);
  }


  // const float x1 = 8.0f;
  // const float y1 = 6.0f;
  // const float z1 = 6.0f;
  // const float e1 = 0.0f;

  // const float x2 = 8.0f;
  // const float y2 = -6.0f;
  // const float z2 = 6.0f;
  // const float e2 = 0.0f;

  // const float x3 = 8.0f;
  // const float y3 = -6.0f;
  // const float z3 = -6.0f;
  // const float e3 = 0.0f;

  // const float x4 = 8.0f;
  // const float y4 = 6.0f;
  // const float z4 = -6.0f;
  // const float e4 = 0.0f;

  // moveBetweenXYZ(x1, y1, z1, e1, x2, y2, z2, e2);
  // delay(500);

  // moveBetweenXYZ(x2, y2, z2, e2, x3, y3, z3, e3);
  // delay(500);

  // moveBetweenXYZ(x3, y3, z3, e3, x4, y4, z4, e4);
  // delay(500);

  // moveBetweenXYZ(x4, y4, z4, e4, x1, y1, z1, e1);
  // delay(500);
}