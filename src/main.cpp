#include <Arduino.h>
#include "Config.h"

unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

float PWM1 = 0;
float PWM2 = 0;
float PWM3 = 0;
float setPercentageSpeed = 0;
int setSpeed = 0;
float maxCalcPWM = 0;
float calcPWM1 = 0;
float calcPWM2 = 0;
float calcPWM3 = 0;

// you can enable debug logging to Serial at 115200
// #define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Omniwheel"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

#include <RemoteXY.h>

// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 53 bytes
    {255, 4, 0, 4, 0, 46, 0, 17, 0, 0, 0, 31, 1, 106, 200, 1, 1, 4, 0, 3,
     22, 6, 66, 24, 131, 105, 26, 5, 12, 48, 60, 60, 53, 106, 26, 31, 4, 85, 83, 15,
     111, 0, 105, 26, 67, 46, 179, 27, 13, 5, 8, 31, 4};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  uint8_t select_01;    // =0 if select position A, =1 if position B, =2 if position C, ...
  int8_t joystick_01_x; // from -100 to 100}
  int8_t joystick_01_y; // from -100 to 100}
  int8_t slider_01;     // =0..100 slider position

  // output variables
  char label[4]; // string UTF8 end zero

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

int roundTo10(int n)
{
  // Smaller multiple
  int a = (n / 10) * 10;

  // Larger multiple
  int b = a + 10;

  // Return of closest of two
  return (n - a > b - n) ? b : a;
}

int roundTo20(int n)
{
  int a = (n / 20) * 20;
  int b = a + 20;
  return (n - a > b - n) ? b : a;
}

void STOP()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  digitalWrite(MOTOR3_PIN1, LOW);
  digitalWrite(MOTOR3_PIN2, LOW);
}

void setMotorsPWM()
{
  if (PWM1 > 0)
  {
    analogWrite(MOTOR1_PIN1, abs(calcPWM1));
    digitalWrite(MOTOR1_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR1_PIN1, LOW);
    analogWrite(MOTOR1_PIN2, abs(calcPWM1));
  }

  if (PWM2 > 0)
  {
    analogWrite(MOTOR2_PIN1, abs(calcPWM2));
    digitalWrite(MOTOR2_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR2_PIN1, LOW);
    analogWrite(MOTOR2_PIN2, abs(calcPWM2));
  }

  if (PWM3 > 0)
  {
    analogWrite(MOTOR3_PIN1, abs(calcPWM3));
    digitalWrite(MOTOR3_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR3_PIN1, LOW);
    analogWrite(MOTOR3_PIN2, abs(calcPWM3));
  }
}

void calcMotorsSpeed(float x, float y, float omega)
{
  if (x != 0 || y != 0)
  {

    // Serial.println(String(x) + ":" + String(y));
    PWM1 = x * M11 + y * M12 + omega * M13;
    PWM2 = x * M21 + y * M22 + omega * M23;
    PWM3 = x * M31 + y * M32 + omega * M33;
    // Serial.println(String(PWM1) + " " + String(PWM2) + " " + String(PWM3));

    setPercentageSpeed = max(abs(x), abs(y));
    // Serial.println("Set speed: " + String(setPercentageSpeed));

    setSpeed = 255 * setPercentageSpeed;
    // Serial.println("Set PWM speed: " + String(setSpeed));

    maxCalcPWM = max(abs(PWM1), max(abs(PWM2), abs(PWM3)));
    // Serial.println("Max calc PWM: " + String(maxCalcPWM));

    calcPWM1 = map(abs(PWM1) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    calcPWM2 = map(abs(PWM2) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    calcPWM3 = map(abs(PWM3) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    // Serial.println("Calc PWM: " + String(calcPWM1) + " " + String(calcPWM2) + " " + String(calcPWM3));

    setMotorsPWM();
  }
  else
  {
    STOP();
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);

  RemoteXY_Init();
  STOP();
  Serial.println("Setup done");
}

void loop()
{
  RemoteXY_Handler();
  if (millis() - lastTime > 100)
  {
    lastTime = millis();

    calcMotorsSpeed((float)roundTo20(RemoteXY.joystick_01_x) / 100, (float)roundTo20(RemoteXY.joystick_01_y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
  }

  if (millis() - lastTime2 > 500)
  {
    lastTime2 = millis();
    float result = map((float)RemoteXY.slider_01, 0, 100, 0, 255);
    Serial.println("Result: " + String(result));
    analogWrite(MOTOR1_PIN1, abs(result));
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR2_PIN1, abs(result));
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR3_PIN1, abs(result));
    digitalWrite(MOTOR3_PIN2, LOW);
    dtostrf(result, 0, 1, RemoteXY.label);
  }
}
