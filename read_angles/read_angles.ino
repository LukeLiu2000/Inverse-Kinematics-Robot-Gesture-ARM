#include <DynamixelShield.h>

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
// 舵机ID
// ------------------------------
const uint8_t ID_6 = 6;
const uint8_t ID_8 = 8;
const uint8_t ID_2 = 2;
const uint8_t ID_1 = 1;

// ------------------------------
// 偏置（统一在这里修改）
// ------------------------------
const float BIAS_6 = -150.0f;
const float BIAS_8 = -60.0f; 
const float BIAS_2 = -60.0f;
const float BIAS_1 = 0.0f;

// ------------------------------
// 读取角度并加偏置
// ------------------------------
float readAngleWithBias(uint8_t id, float bias)
{
  float raw = dxl.getPresentPosition(id, UNIT_DEGREE);
  return raw + bias;
}

void setup()
{
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Start reading servo angles (with bias)...");
}

void loop()
{
  float angle6 = readAngleWithBias(ID_6, BIAS_6);
  float angle8 = readAngleWithBias(ID_8, BIAS_8);
  float angle2 = readAngleWithBias(ID_2, BIAS_2);
  float angle1 = readAngleWithBias(ID_1, BIAS_1);

  DEBUG_SERIAL.print("ID6: ");
  DEBUG_SERIAL.print(angle6, 2);
  DEBUG_SERIAL.print(" | ");

  DEBUG_SERIAL.print("ID8: ");
  DEBUG_SERIAL.print(angle8, 2);
  DEBUG_SERIAL.print(" | ");

  DEBUG_SERIAL.print("ID2: ");
  DEBUG_SERIAL.print(angle2, 2);
  DEBUG_SERIAL.print(" | ");

  DEBUG_SERIAL.print("ID1: ");
  DEBUG_SERIAL.print(angle1, 2);
  DEBUG_SERIAL.println();

  delay(100);  // 10Hz刷新
}