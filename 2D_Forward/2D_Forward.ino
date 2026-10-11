#include <DynamixelShield.h>
#include <math.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// ------------------------------
// 机械臂连杆长度
// ------------------------------
const float L1 = 10.0f;
const float L2 = 10.0f;

// ------------------------------
// 舵机ID
// ------------------------------
const uint8_t ID_8 = 8;
const uint8_t ID_2 = 2;

// ------------------------------
// 偏置
// ------------------------------
const float BIAS_8 = -60.0f;
const float BIAS_2 = -60.0f;

// ------------------------------
// 只负责读取角度并加偏置
// ------------------------------
float readAngleWithBias(uint8_t id, float bias)
{
  float raw = dxl.getPresentPosition(id, UNIT_DEGREE);
  return raw + bias;
}

// ------------------------------
// 正向运动学
// 输入：alpha_deg, beta_deg
// 输出：x, y
// 角度单位：deg
//
// 这里修改为：
// theta1 = pi - alpha
// theta2 = beta - alpha
// ------------------------------
void forwardKinematics(float alpha_deg, float beta_deg, float &x, float &y)
{
  float alpha_rad = alpha_deg * PI / 180.0f;
  float beta_rad  = beta_deg  * PI / 180.0f;

  float theta1 = PI - alpha_rad;
  float theta2 = beta_rad - alpha_rad;

  x = L1 * cos(theta1) + L2 * cos(theta2);
  y = L1 * sin(theta1) + L2 * sin(theta2);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Start reading servo angles (with bias) and computing x,y...");
}

void loop()
{
  // 读取 8 号舵机，作为 alpha
  float alpha_deg = readAngleWithBias(ID_8, BIAS_8);

  // 读取 2 号舵机，作为 beta
  float beta_deg = readAngleWithBias(ID_2, BIAS_2);

  // 计算 x, y
  float x = 0.0f;
  float y = 0.0f;
  forwardKinematics(alpha_deg, beta_deg, x, y);

  // 串口输出
  DEBUG_SERIAL.print("alpha(ID8): ");
  DEBUG_SERIAL.print(alpha_deg, 2);
  DEBUG_SERIAL.print(" deg | beta(ID2): ");
  DEBUG_SERIAL.print(beta_deg, 2);
  DEBUG_SERIAL.print(" deg | x: ");
  DEBUG_SERIAL.print(x, 2);
  DEBUG_SERIAL.print(" | y: ");
  DEBUG_SERIAL.println(y, 2);

  delay(100);
}