#include <Arduino.h>
#include "Config.h"

#define DEBUG 0

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

unsigned long lastTime = 0;
unsigned long lastTime2 = 0;

float PWM1 = 0;
float PWM2 = 0;
float PWM3 = 0;
float PWM21 = 0;
float PWM22 = 0;
float PWM23 = 0;
float setPercentageSpeed = 0;
float setPercentageSpeed2 = 0;
int setSpeed = 0;
int setSpeed2 = 0;
float maxCalcPWM = 0;
float maxCalcPWM2 = 0;
float calcPWM1 = 0;
float calcPWM2 = 0;
float calcPWM3 = 0;
float calcPWM21 = 0;
float calcPWM22 = 0;
float calcPWM23 = 0;

// RPM variable
unsigned long M1t1 = 0, M1t2 = 0, M1t = 0;
unsigned long M2t1 = 0, M2t2 = 0, M2t = 0;
unsigned long M3t1 = 0, M3t2 = 0, M3t = 0;
bool M1MeasDone = 0, M2MeasDone = 0, M3MeasDone = 0;
unsigned long M1previousInterrupt = 0;
unsigned long M2previousInterrupt = 0;
unsigned long M3previousInterrupt = 0;

String inputString = "";     // a String to hold incoming data
bool stringComplete = false; // whether the string is complete

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
uint8_t RemoteXY_CONF[] = // 45 bytes
    {255, 3, 0, 6, 0, 38, 0, 17, 0, 0, 0, 31, 1, 106, 200, 1, 1, 3, 0, 5,
     15, 40, 60, 60, 53, 106, 26, 31, 4, 85, 83, 15, 111, 0, 105, 26, 67, 30, 152, 40,
     10, 4, 16, 26, 6};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  int8_t joystick_01_x; // from -100 to 100}
  int8_t joystick_01_y; // from -100 to 100}
  int8_t slider_01;     // =0..100 slider position

  // output variables
  char sliderValue[6]; // string UTF8 end zero

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/// @brief Interrupt handler to measure RPM
void IRAM_ATTR isrM1()
{
  if (millis() - M1previousInterrupt >= 70)
  {
    M1previousInterrupt = millis();
    if (M1MeasDone)
    {
      M1t2 = micros();
      M1t = M1t2 - M1t1;
      M1MeasDone = 0;
    }
    else
    {
      M1t1 = micros();
      M1MeasDone = 1;
    }
  }
}

void IRAM_ATTR isrM2()
{
  if (millis() - M2previousInterrupt >= 70)
  {
    M2previousInterrupt = millis();
    if (M2MeasDone)
    {
      M2t2 = micros();
      M2t = M2t2 - M2t1;
      M2MeasDone = 0;
    }
    else
    {
      M2t1 = micros();
      M2MeasDone = 1;
    }
  }
}

void IRAM_ATTR isrM3()
{
  if (millis() - M3previousInterrupt >= 70)
  {
    M3previousInterrupt = millis();
    if (M3MeasDone)
    {
      M3t2 = micros();
      M3t = M3t2 - M3t1;
      M3MeasDone = 0;
    }
    else
    {
      M3t1 = micros();
      M3MeasDone = 1;
    }
  }
}

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

void setMotorsPWM2()
{
  if (PWM21 > 0)
  {
    analogWrite(MOTOR1_PIN1, abs(calcPWM21));
    digitalWrite(MOTOR1_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR1_PIN1, LOW);
    analogWrite(MOTOR1_PIN2, abs(calcPWM21));
  }

  if (PWM22 > 0)
  {
    analogWrite(MOTOR2_PIN1, abs(calcPWM22));
    digitalWrite(MOTOR2_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR2_PIN1, LOW);
    analogWrite(MOTOR2_PIN2, abs(calcPWM22));
  }

  if (PWM23 > 0)
  {
    analogWrite(MOTOR3_PIN1, abs(calcPWM23));
    digitalWrite(MOTOR3_PIN2, LOW);
  }
  else
  {
    digitalWrite(MOTOR3_PIN1, LOW);
    analogWrite(MOTOR3_PIN2, abs(calcPWM23));
  }
}

void calcMotorsSpeed(float x, float y, float omega)
{
  if (x != 0 || y != 0)
  {

    // debugln(String(x) + ":" + String(y));
    PWM1 = x * M11 + y * M12 + omega * M13;
    PWM2 = x * M21 + y * M22 + omega * M23;
    PWM3 = x * M31 + y * M32 + omega * M33;
    // debugln(String(PWM1) + " " + String(PWM2) + " " + String(PWM3));

    setPercentageSpeed = max(abs(x), abs(y));
    // debugln("Set speed: " + String(setPercentageSpeed));

    // setSpeed = 255 * setPercentageSpeed;
    setSpeed = map(setPercentageSpeed * 100, 0, 100, PWM_THRESHOLD, 255);
    // debugln("Set PWM speed: " + String(setSpeed));

    maxCalcPWM = max(abs(PWM1), max(abs(PWM2), abs(PWM3)));
    // debugln("Max calc PWM: " + String(maxCalcPWM));

    calcPWM1 = map(abs(PWM1) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    calcPWM2 = map(abs(PWM2) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    calcPWM3 = map(abs(PWM3) * 100, 0, maxCalcPWM * 100, PWM_THRESHOLD, setSpeed);
    debugln("Calc PWM: " + String(calcPWM1) + " " + String(calcPWM2) + " " + String(calcPWM3));

    // calcPWM1 = PWM_Const_2[(int)calcPWM1];
    // calcPWM2 = PWM_Const_2[(int)calcPWM2];
    // calcPWM3 = PWM_Const_2[(int)calcPWM3];
    // calcPWM1 = 0; // 127-135
    // calcPWM2 = 0;   // 137-156
    // calcPWM3 = 0;   // 146-151

    debugln("Cacl new PWM: " + String(calcPWM1) + " " + String(calcPWM2) + " " + String(calcPWM3));

    setMotorsPWM();
  }
  else
  {
    STOP();
  }
}

void calcMotorsSpeed2(float x_dot, float y_dot, float theta_dot)
{
  if (x_dot != 0 || y_dot != 0)
  {
    float R = 1; // radius of wheel
    PWM23 = -R * theta_dot + x_dot;
    PWM22 = -R * theta_dot - 0.5 * x_dot - sin(PI / 3.0) * y_dot;
    PWM21 = -R * theta_dot - 0.5 * x_dot + sin(PI / 3.0) * y_dot;

    setPercentageSpeed2 = max(abs(x_dot), abs(y_dot));
    // debugln("Set speed: " + String(setPercentageSpeed));

    setSpeed2 = map(setPercentageSpeed2 * 100, 0, 100, PWM_THRESHOLD, 255);
    // debugln("Set PWM speed: " + String(setSpeed));

    maxCalcPWM2 = max(abs(PWM21), max(abs(PWM22), abs(PWM23)));
    // debugln("Max calc PWM: " + String(maxCalcPWM));

    calcPWM21 = map(abs(PWM21) * 100, 0, maxCalcPWM2 * 100, PWM_THRESHOLD, setSpeed2);
    calcPWM22 = map(abs(PWM22) * 100, 0, maxCalcPWM2 * 100, PWM_THRESHOLD, setSpeed2);
    calcPWM23 = map(abs(PWM23) * 100, 0, maxCalcPWM2 * 100, PWM_THRESHOLD, setSpeed2);
    debugln("Calc PWM2: " + String(calcPWM21) + " " + String(calcPWM22) + " " + String(calcPWM23));
    debugln("PWM: " + String(PWM21) + " " + String(PWM22) + " " + String(PWM23));

    setMotorsPWM2();
  }
  else
  {
    STOP();
  }
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);

  pinMode(MOTOR1_RPM_PIN, INPUT);
  pinMode(MOTOR2_RPM_PIN, INPUT);
  pinMode(MOTOR3_RPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_RPM_PIN), isrM1, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_RPM_PIN), isrM2, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_RPM_PIN), isrM3, FALLING);

  RemoteXY_Init();
  STOP();

  debugln("Setup done");
}

void loop()
{
  RemoteXY_Handler();
  if (stringComplete)
  {
    String s = inputString;
    // clear the string:
    inputString = "";
    stringComplete = false;

    int result = s.toInt();
    debugln("Result: " + String(result));
    analogWrite(MOTOR1_PIN1, abs(result));
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR2_PIN1, abs(result));
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR3_PIN1, abs(result));
    digitalWrite(MOTOR3_PIN2, LOW);
    dtostrf(result, 0, 1, RemoteXY.sliderValue);
  }

  if (millis() - lastTime > 100)
  {
    lastTime = millis();

    calcMotorsSpeed((float)roundTo20(RemoteXY.joystick_01_x) / 100, (float)roundTo20(RemoteXY.joystick_01_y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
    //  calcMotorsSpeed2((float)roundTo20(RemoteXY.joystick_01_x) / 100, (float)roundTo20(RemoteXY.joystick_01_y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
  }

  if (millis() - lastTime2 > 100)
  {
    lastTime2 = millis();
    /* float result = map((float)RemoteXY.slider_01, 0, 100, 0, 255);
    debugln("Result: " + String(result));
    analogWrite(MOTOR1_PIN1, abs(result));
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR2_PIN1, abs(result));
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR3_PIN1, abs(result));
    digitalWrite(MOTOR3_PIN2, LOW);
    dtostrf(result, 0, 1, RemoteXY.sliderValue); */
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') // enter key
    {
      stringComplete = true;
    }
  }
}
