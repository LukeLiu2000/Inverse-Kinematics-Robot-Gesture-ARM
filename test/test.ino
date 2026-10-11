#include <DynamixelShield.h>
#include <math.h>
#include <string.h>

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
// --------------------------------------------------
void inverseKinematics(float x, float y, float &alpha_deg, float &beta_deg)
{
  float r = sqrt(x * x + y * y);

  if (r > (L1 + L2) || r < fabs(L1 - L2) || r == 0.0f)
  {
    alpha_deg = NAN;
    beta_deg = NAN;
    return;
  }

  float cosBeta = (L1 * L1 + L2 * L2 - r * r) / (2.0f * L1 * L2);
  if (cosBeta > 1.0f) cosBeta = 1.0f;
  if (cosBeta < -1.0f) cosBeta = -1.0f;

  float beta = acos(cosBeta);

  float phi = atan2(y, x);

  float cosGamma = (L1 * L1 + r * r - L2 * L2) / (2.0f * L1 * r);
  if (cosGamma > 1.0f) cosGamma = 1.0f;
  if (cosGamma < -1.0f) cosGamma = -1.0f;

  float gamma = acos(cosGamma);

  float theta1 = phi + gamma;
  float alpha = PI - theta1;

  alpha_deg = alpha * 180.0f / PI;
  beta_deg = beta * 180.0f / PI;
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
// 读取一行串口命令：X,Y,Z,error
// 返回 true 表示成功解析到一组新数据
// --------------------------------------------------
bool readXYZErrorFromSerial(float &X, float &Y, float &Z, float &error_deg)
{
  static char buffer[80];
  static uint8_t idx = 0;

  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      buffer[idx] = '\0';
      idx = 0;

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

    if (idx < sizeof(buffer) - 1)
    {
      buffer[idx++] = c;
    }
    else
    {
      idx = 0;
    }
  }

  return false;
}

void setup()
{
  Serial.begin(115200);     // MATLAB 通信串口
  DEBUG_SERIAL.begin(115200);

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

  DEBUG_SERIAL.println("System ready.");
}

void loop()
{
  float X, Y, Z, error_deg;

  if (readXYZErrorFromSerial(X, Y, Z, error_deg))
  {
    moveToXYZ(X, Y, Z, error_deg);
  }
}