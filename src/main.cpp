#include <Arduino.h>
#include "Config.h"
#include <LPF.h>

#define DEBUG 0

#if DEBUG == 1
#define debug(x)   // Serial.print(x)
#define debugln(x) // Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

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

// RPM variable
unsigned long M1t1 = 0, M1t2 = 0, M1t = 0;
unsigned long M2t1 = 0, M2t2 = 0, M2t = 0;
unsigned long M3t1 = 0, M3t2 = 0, M3t = 0;
bool M1MeasDone = 0, M2MeasDone = 0, M3MeasDone = 0;
unsigned long M1previousInterrupt = 0;
unsigned long M2previousInterrupt = 0;
unsigned long M3previousInterrupt = 0;
unsigned long previousM1T1 = 0;
unsigned long previousM2T1 = 0;
unsigned long previousM3T1 = 0;
int Motor1_RPM = 0;
int Motor2_RPM = 0;
int Motor3_RPM = 0;

LPF LPF_M1(1);
LPF LPF_M2(1);
LPF LPF_M3(1);
int PFV_M1 = 0;
int PFV_M2 = 0;
int PFV_M3 = 0;

String str;
String m1OutputText;
String m2OutputText;
String m3OutputText;

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

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 63 bytes
    {255, 3, 0, 63, 0, 56, 0, 17, 0, 0, 0, 27, 1, 106, 200, 1, 1, 5, 0, 5,
     22, 27, 60, 60, 53, 12, 26, 31, 67, 18, 168, 55, 13, 4, 31, 26, 21, 67, 18, 150,
     55, 13, 4, 31, 26, 21, 67, 18, 134, 55, 13, 4, 31, 26, 21, 4, 83, 80, 16, 110,
     0, 2, 26};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t slider_01;     // from 0 to 100

  // output variables
  char M3[21]; // string UTF8 end zero
  char M2[21]; // string UTF8 end zero
  char M1[21]; // string UTF8 end zero

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/// @brief Interrupt handler to measure RPM for motor 1
void IRAM_ATTR isrM1()
{
  if (millis() - M1previousInterrupt >= PREVIOUS_INTERRUPT_LIMIT)
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

/// @brief Interrupt handler to measure RPM for motor 2
void IRAM_ATTR isrM2()
{
  if (millis() - M2previousInterrupt >= PREVIOUS_INTERRUPT_LIMIT)
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

/// @brief Interrupt handler to measure RPM for motor 3
void IRAM_ATTR isrM3()
{
  if (millis() - M3previousInterrupt >= PREVIOUS_INTERRUPT_LIMIT)
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

/// @brief Calculate RPM for motor 1
void calcMotor1RPM()
{
  if (M1t != 0)
  {
    Motor1_RPM = (60000000) / (M1t * ENCODER_N);
    if (Motor1_RPM != 0)
    {
      if (millis() - (previousM1T1 / 1000) >= ((((60 * 1000) / (Motor1_RPM * ENCODER_N)) * 2) + 500))
      {
        if (previousM1T1 == M1t1)
        {
          M1t = 0;
          Motor1_RPM = 0;
        }
        previousM1T1 = M1t1;
      }
    }
  }
}

/// @brief Calculate RPM for motor 2
void calcMotor2RPM()
{
  if (M2t != 0)
  {
    Motor2_RPM = (60000000) / (M2t * ENCODER_N);
    if (Motor2_RPM != 0)
    {
      if (millis() - (previousM2T1 / 1000) >= ((((60 * 1000) / (Motor2_RPM * ENCODER_N)) * 2) + 500))
      {
        if (previousM2T1 == M2t1)
        {
          M2t = 0;
          Motor2_RPM = 0;
        }
        previousM2T1 = M2t1;
      }
    }
  }
}

/// @brief Calculate RPM for motor 3
void calcMotor3RPM()
{
  if (M3t != 0)
  {
    Motor3_RPM = (60000000) / (M3t * ENCODER_N);
    if (Motor3_RPM != 0)
    {
      if (millis() - (previousM3T1 / 1000) >= ((((60 * 1000) / (Motor3_RPM * ENCODER_N)) * 2) + 500))
      {
        if (previousM3T1 == M3t1)
        {
          M3t = 0;
          Motor3_RPM = 0;
        }
        previousM3T1 = M3t1;
      }
    }
  }
}

/// @brief round Input to nearest multiple of 10
/// @param n input number
/// @return round to nearest multiple of 10
int roundTo10(int n)
{
  // Smaller multiple
  int a = (n / 10) * 10;

  // Larger multiple
  int b = a + 10;

  // Return of closest of two
  return (n - a > b - n) ? b : a;
}

/// @brief round Input to nearest multiple of 20
/// @param n input number
/// @return round to nearest multiple of 20
int roundTo20(int n)
{
  int a = (n / 20) * 20;
  int b = a + 20;
  return (n - a > b - n) ? b : a;
}

/// @brief Stop all motors
void STOP()
{
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  digitalWrite(MOTOR3_PIN1, LOW);
  digitalWrite(MOTOR3_PIN2, LOW);
}

/// @brief Set PWM to motors
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

/// @brief Calculate motor speed from joystick input
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
    // debugln("Calc PWM: " + String(calcPWM1) + " " + String(calcPWM2) + " " + String(calcPWM3));

    setMotorsPWM();
  }
  else
  {
    STOP();
  }
}

/// @brief Setup function
void setup()
{
  delay(1000);
  // Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY, 1, false);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR3_PIN1, OUTPUT);
  pinMode(MOTOR3_PIN2, OUTPUT);

  pinMode(MOTOR1_RPM_PIN, INPUT);
  pinMode(MOTOR2_RPM_PIN, INPUT);

  pinMode(MOTOR3_RPM_PIN, FUNCTION_3);
  pinMode(MOTOR3_RPM_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_RPM_PIN), isrM1, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_RPM_PIN), isrM2, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_RPM_PIN), isrM3, FALLING);

  RemoteXY_Init();
  STOP();

  debugln("Setup done");
}

/// @brief Main loop
void loop()
{
  RemoteXY_Handler();

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
    analogWrite(MOTOR1_PIN1, abs(result));
    digitalWrite(MOTOR1_PIN2, LOW);
    analogWrite(MOTOR2_PIN1, abs(result));
    digitalWrite(MOTOR2_PIN2, LOW);
    analogWrite(MOTOR3_PIN1, abs(result));
    digitalWrite(MOTOR3_PIN2, LOW);*/
  }
  calcMotor1RPM();
  calcMotor2RPM();
  calcMotor3RPM();

  // str = String(Motor1_RPM) + ";" + String(Motor2_RPM) + ";" + String(Motor3_RPM);

  PFV_M1 = LPF_M1.Step(Motor1_RPM);
  PFV_M2 = LPF_M2.Step(Motor2_RPM);
  PFV_M3 = LPF_M3.Step(Motor3_RPM);

  m1OutputText = String(calcPWM1, 0) + "\t" + String(PFV_M1);
  m2OutputText = String(calcPWM2, 0) + "\t" + String(PFV_M2);
  m3OutputText = String(calcPWM3, 0) + "\t" + String(PFV_M3);

  strcpy(RemoteXY.M1, m1OutputText.c_str());
  strcpy(RemoteXY.M2, m2OutputText.c_str());
  strcpy(RemoteXY.M3, m3OutputText.c_str());
}
