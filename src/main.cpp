#include <Arduino.h>
#include "Config.h"
#include <CheapStepper.h>

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

CheapStepper stepper1(STEPPER1_PIN1, STEPPER1_PIN2, STEPPER1_PIN3, STEPPER1_PIN4);
CheapStepper stepper2(STEPPER2_PIN1, STEPPER2_PIN2, STEPPER2_PIN3, STEPPER2_PIN4);
CheapStepper stepper3(STEPPER3_PIN1, STEPPER3_PIN2, STEPPER3_PIN3, STEPPER3_PIN4);

boolean stepper1moveClockwise = true;
boolean stepper2moveClockwise = true;
boolean stepper3moveClockwise = true;

unsigned long lastTime = 0;

unsigned long testlastTime = 0;
int RPM = 0;

float PWM1 = 0;
float PWM2 = 0;
float PWM3 = 0;
float setPercentageSpeed = 0;
int setSpeed = 0;
float maxCalcPWM = 0;
float calcPWM1 = 0;
float calcPWM2 = 0;
float calcPWM3 = 0;

float realPWM1 = 0;
float realPWM2 = 0;
float realPWM3 = 0;

// RPM variable

String str;
String m1OutputText;
String m2OutputText;
String m3OutputText;

// Filter variables
const int SMOOTHING_WINDOW_SIZE = 10;
int _samplesM1[SMOOTHING_WINDOW_SIZE];
int _curReadIndexM1 = 0;
int _sampleTotalM1 = 0;
int filterResultM1 = 0;
int _samplesM2[SMOOTHING_WINDOW_SIZE];
int _curReadIndexM2 = 0;
int _sampleTotalM2 = 0;
int filterResultM2 = 0;
int _samplesM3[SMOOTHING_WINDOW_SIZE];
int _curReadIndexM3 = 0;
int _sampleTotalM3 = 0;
int filterResultM3 = 0;

float _ewmaAlpha = 0.01; // the EWMA alpha value (α)

// you can enable debug logging to Serial at 115200
// #define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__WIFI_POINT

#include <WiFi.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "Omniwheel"
#define REMOTEXY_WIFI_PASSWORD ""
#define REMOTEXY_SERVER_PORT 6377

#include <RemoteXY.h>

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 38 bytes
    {255, 2, 0, 0, 0, 31, 0, 19, 0, 0, 0, 79, 109, 110, 105, 119, 104, 101, 101, 108,
     0, 156, 1, 106, 200, 1, 1, 1, 0, 5, 15, 58, 81, 81, 52, 16, 26, 24};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  int8_t x; // from -100 to 100
  int8_t y; // from -100 to 100

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)
/*
void smoothMovingAverageFilterM1()
{
  _sampleTotalM1 = _sampleTotalM1 - _samplesM1[_curReadIndexM1];
  _samplesM1[_curReadIndexM1] = Motor1_RPM;
  _sampleTotalM1 = _sampleTotalM1 + _samplesM1[_curReadIndexM1];
  _curReadIndexM1++;

  if (_curReadIndexM1 >= SMOOTHING_WINDOW_SIZE)
  {
    _curReadIndexM1 = 0;
  }
  filterResultM1 = _sampleTotalM1 / SMOOTHING_WINDOW_SIZE;
}

void smoothMovingAverageFilterM2()
{
  _sampleTotalM2 = _sampleTotalM2 - _samplesM2[_curReadIndexM2];
  _samplesM2[_curReadIndexM2] = Motor2_RPM;
  _sampleTotalM2 = _sampleTotalM2 + _samplesM2[_curReadIndexM2];
  _curReadIndexM2++;

  if (_curReadIndexM2 >= SMOOTHING_WINDOW_SIZE)
  {
    _curReadIndexM2 = 0;
  }
  filterResultM2 = _sampleTotalM2 / SMOOTHING_WINDOW_SIZE;
}

void smoothMovingAverageFilterM3()
{
  _sampleTotalM3 = _sampleTotalM3 - _samplesM3[_curReadIndexM3];
  _samplesM3[_curReadIndexM3] = Motor3_RPM;
  _sampleTotalM3 = _sampleTotalM3 + _samplesM3[_curReadIndexM3];
  _curReadIndexM3++;

  if (_curReadIndexM3 >= SMOOTHING_WINDOW_SIZE)
  {
    _curReadIndexM3 = 0;
  }
  filterResultM3 = _sampleTotalM3 / SMOOTHING_WINDOW_SIZE;
}
*/
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
  calcPWM1 = 0;
  calcPWM2 = 0;
  calcPWM3 = 0;
  stepper1.stop();
  stepper2.stop();
  stepper3.stop();
}

/// @brief Set PWM to motors
void setMotorsPWM()
{
  if (PWM1 > 0)
  {
    stepper1.move(calcPWM1, true);
  }
  else
  {
    stepper1.move(calcPWM1, false);
  }

  if (PWM2 > 0)
  {
    stepper2.move(calcPWM2, true);
  }
  else
  {
    stepper2.move(calcPWM2, false);
  }

  if (PWM3 > 0)
  {
    stepper3.move(calcPWM3, true);
  }
  else
  {
    stepper3.move(calcPWM3, false);
  }
}

/// @brief Calculate motor speed from joystick input
void calcMotorsSpeed(float x, float y, float omega)
{
  if (x != 0 || y != 0)
  {

    debugln(String(x) + ":" + String(y));
    PWM1 = x * M11 + y * M12 + omega * M13;
    PWM2 = x * M21 + y * M22 + omega * M23;
    PWM3 = x * M31 + y * M32 + omega * M33;
    debugln(String(PWM1) + " " + String(PWM2) + " " + String(PWM3));

    setPercentageSpeed = max(abs(x), abs(y));
    debugln("Set speed: " + String(setPercentageSpeed));

    // setSpeed = 255 * setPercentageSpeed;
    setSpeed = map(setPercentageSpeed * 100, 0, 100, 0, STEPPER_MAX_SPEED);
    debugln("Set PWM speed: " + String(setSpeed));

    maxCalcPWM = max(abs(PWM1), max(abs(PWM2), abs(PWM3)));
    debugln("Max calc PWM: " + String(maxCalcPWM));

    calcPWM1 = map(abs(PWM1) * 100, 0, maxCalcPWM * 100, 0, setSpeed);
    calcPWM2 = map(abs(PWM2) * 100, 0, maxCalcPWM * 100, 0, setSpeed);
    calcPWM3 = map(abs(PWM3) * 100, 0, maxCalcPWM * 100, 0, setSpeed);
    debugln("Calc PWM: " + String(calcPWM1) + " " + String(calcPWM2) + " " + String(calcPWM3));

    debugln("__________________________");

    setMotorsPWM();
  }
  else
  {
    STOP();
  }
}

void calcStepperSpeed(float x, float y, float omega)
{
  if (x != 0 || y != 0)
  {
    PWM1 = x * M11 + y * M12 + omega * M13;
    PWM2 = x * M21 + y * M22 + omega * M23;
    PWM3 = x * M31 + y * M32 + omega * M33;

    Serial.println("PWM1: " + String(PWM1) + " PWM2: " + String(PWM2) + " PWM3: " + String(PWM3));

    // TODO: calculate PWM values for stepper motors
    calcPWM1 = abs(PWM1) * 255;
    calcPWM2 = abs(PWM2) * 255;
    calcPWM3 = abs(PWM3) * 255;
  }
  else
  {
    STOP();
  }
}
/*
void exponentialMovingAverageFilterM1()
{
  filterResultM1 = (_ewmaAlpha * Motor1_RPM) + (1 - _ewmaAlpha) * filterResultM1;
}

void exponentialMovingAverageFilterM2()
{
  filterResultM2 = (_ewmaAlpha * Motor2_RPM) + (1 - _ewmaAlpha) * filterResultM2;
}

void exponentialMovingAverageFilterM3()
{
  filterResultM3 = (_ewmaAlpha * Motor3_RPM) + (1 - _ewmaAlpha) * filterResultM3;
}*/

/// @brief Setup function
void setup()
{
  delay(1000);
  Serial.begin(115200);

  for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++)
  {
    _samplesM1[i] = 0;
    _samplesM2[i] = 0;
    _samplesM3[i] = 0;
  }

  RemoteXY_Init();
  STOP();
}

/// @brief Main loop
void loop()
{
  RemoteXY_Handler();

  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (millis() - lastTime > 100)
  {
    lastTime = millis();
    // calcMotorsSpeed((float)roundTo20(RemoteXY.joystick_01_x) / 100, (float)roundTo20(RemoteXY.joystick_01_y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
    // get coordinates from joystick
    calcMotorsSpeed((float)roundTo20(RemoteXY.x) / 100, (float)roundTo20(RemoteXY.y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
  }

  // apply filter on RPM values
  // smoothMovingAverageFilterM1();
  // smoothMovingAverageFilterM2();
  // smoothMovingAverageFilterM3();

  // exponentialMovingAverageFilterM1();
  // exponentialMovingAverageFilterM2();
  // exponentialMovingAverageFilterM3();

  /*
    // make output text
    m1OutputText = String(calcPWM1, 0) + "\t" + String(realPWM1, 0) + "\t" + String(filterResultM1);
    m2OutputText = String(calcPWM2, 0) + "\t" + String(realPWM2, 0) + "\t" + String(filterResultM2);
    m3OutputText = String(calcPWM3, 0) + "\t" + String(realPWM3, 0) + "\t" + String(filterResultM3);

    // send output text to RemoteXY
    strcpy(RemoteXY.M1, m1OutputText.c_str());
    strcpy(RemoteXY.M2, m2OutputText.c_str());
    strcpy(RemoteXY.M3, m3OutputText.c_str());
    */
}
