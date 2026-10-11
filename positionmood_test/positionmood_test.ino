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

// 在这里改舵机ID
const uint8_t TARGET_ID = 2;

// 初始化：切到位置模式
void initPositionMode(uint8_t id)
{
  dxl.torqueOff(id);
  delay(50);

  dxl.setOperatingMode(id, OP_POSITION);
  delay(50);

  dxl.torqueOn(id);
  delay(50);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (dxl.ping(TARGET_ID))
  {
    DEBUG_SERIAL.println("Servo found");
  }
  else
  {
    DEBUG_SERIAL.println("Servo not found");
  }

  initPositionMode(TARGET_ID);
}

void loop()
{
  dxl.setGoalPosition(TARGET_ID, 30.0, UNIT_DEGREE);
  delay(1000);

  dxl.setGoalPosition(TARGET_ID, 60.0, UNIT_DEGREE);
  delay(1000);
}