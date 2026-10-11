#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#else
  #define DEBUG_SERIAL Serial
#endif

void setup()
{
  DEBUG_SERIAL.begin(57600);
  DEBUG_SERIAL.println("Arduino ready");
}

void loop()
{
  static char buffer[120];
  static uint8_t idx = 0;

  while (DEBUG_SERIAL.available() > 0)
  {
    char c = (char)DEBUG_SERIAL.read();

    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      buffer[idx] = '\0';
      idx = 0;

      float a = 0.0f;
      float b = 0.0f;
      float c2 = 0.0f;
      float d = 0.0f;

      char *token = strtok(buffer, ",");
      if (token == NULL)
      {
        DEBUG_SERIAL.println("PARSE_ERROR");
        continue;
      }
      a = atof(token);

      token = strtok(NULL, ",");
      if (token == NULL)
      {
        DEBUG_SERIAL.println("PARSE_ERROR");
        continue;
      }
      b = atof(token);

      token = strtok(NULL, ",");
      if (token == NULL)
      {
        DEBUG_SERIAL.println("PARSE_ERROR");
        continue;
      }
      c2 = atof(token);

      token = strtok(NULL, ",");
      if (token == NULL)
      {
        DEBUG_SERIAL.println("PARSE_ERROR");
        continue;
      }
      d = atof(token);

      float sum = a + b + c2 + d;

      DEBUG_SERIAL.print("SUM=");
      DEBUG_SERIAL.println(sum, 6);
    }
    else
    {
      if (idx < sizeof(buffer) - 1)
      {
        buffer[idx++] = c;
      }
      else
      {
        idx = 0;
      }
    }
  }
}