#include <Arduino.h>
#include "Config.h"
#include <CheapStepper.h>

#define DEBUG 0

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

TaskHandle_t RemoteXYTask;
TaskHandle_t StepperTask;

static SemaphoreHandle_t xMutexStepper1 = xSemaphoreCreateMutex();
static SemaphoreHandle_t xMutexStepper2 = xSemaphoreCreateMutex();
static SemaphoreHandle_t xMutexStepper3 = xSemaphoreCreateMutex();

CheapStepper stepper1(STEPPER1_PIN1, STEPPER1_PIN2, STEPPER1_PIN3, STEPPER1_PIN4);
CheapStepper stepper2(STEPPER2_PIN1, STEPPER2_PIN2, STEPPER2_PIN3, STEPPER2_PIN4);
CheapStepper stepper3(STEPPER3_PIN1, STEPPER3_PIN2, STEPPER3_PIN3, STEPPER3_PIN4);

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
    // Take mutex
    if (xSemaphoreTake(xMutexStepper1, portMAX_DELAY) == pdTRUE)
    {
      stepper1.move(calcPWM1, false);
      xSemaphoreGive(xMutexStepper1);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
  }
  else
  {
    // Take mutex
    if (xSemaphoreTake(xMutexStepper1, portMAX_DELAY) == pdTRUE)
    {
      stepper1.move(calcPWM1, true);
      xSemaphoreGive(xMutexStepper1);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
  }

  if (PWM2 > 0)
  {
    // Take mutex
    if (xSemaphoreTake(xMutexStepper2, portMAX_DELAY) == pdTRUE)
    {
      stepper2.move(calcPWM2, false);
      xSemaphoreGive(xMutexStepper2);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
  }
  else
  {
    // Take mutex
    if (xSemaphoreTake(xMutexStepper2, portMAX_DELAY) == pdTRUE)
    {
      stepper2.move(calcPWM2, true);
      xSemaphoreGive(xMutexStepper2);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
  }

  if (PWM3 > 0)
  {
    // Take mutex
    if (xSemaphoreTake(xMutexStepper3, portMAX_DELAY) == pdTRUE)
    {
      stepper3.move(calcPWM3, false);
      xSemaphoreGive(xMutexStepper3);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
  }
  else
  {
    // Take mutex
    if (xSemaphoreTake(xMutexStepper3, portMAX_DELAY) == pdTRUE)
    {
      stepper3.move(calcPWM3, true);
      xSemaphoreGive(xMutexStepper3);
    }
    else
    {
      debugln("Failed to take mutex in move");
    }
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

/// @brief RemoteXY task
void RemoteXY_Task(void *pvParameters)
{
  RemoteXY_Init();
  for (;;)
  {
    RemoteXY_Handler();

    if (millis() - lastTime > 100)
    {
      lastTime = millis();
      // calcMotorsSpeed((float)roundTo20(RemoteXY.joystick_01_x) / 100, (float)roundTo20(RemoteXY.joystick_01_y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
      // get coordinates from joystick
      calcMotorsSpeed((float)roundTo10(RemoteXY.x) / 100, (float)roundTo10(RemoteXY.y) / 100, /*RemoteXY.orientation_01_yaw*/ 0);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/// @brief Stepper1 task
void Stepper_Task(void *pvParameters)
{
  for (;;)
  {
    // Take Mutex
    if (xSemaphoreTake(xMutexStepper1, portMAX_DELAY) == pdTRUE)
    {
      stepper1.run();
      xSemaphoreGive(xMutexStepper1);
    }

    if (xSemaphoreTake(xMutexStepper2, portMAX_DELAY) == pdTRUE)
    {
      stepper2.run();
      xSemaphoreGive(xMutexStepper2);
    }

    if (xSemaphoreTake(xMutexStepper3, portMAX_DELAY) == pdTRUE)
    {
      stepper3.run();
      xSemaphoreGive(xMutexStepper3);
    }
  }
}

/// @brief Setup function
void setup()
{
  Serial.begin(115200);

  xMutexStepper1 = xSemaphoreCreateMutex();
  xMutexStepper2 = xSemaphoreCreateMutex();
  xMutexStepper3 = xSemaphoreCreateMutex();

  // create task for RemoteXY
  xTaskCreatePinnedToCore(
      RemoteXY_Task,   /* Task function. */
      "RemoteXY_Task", /* name of task. */
      10000,           /* Stack size of task */
      NULL,            /* parameter of the task */
      1,               /* priority of the task */
      &RemoteXYTask,
      0); /* Task handle to keep track of created task */

  // create task for Stepper1
  xTaskCreatePinnedToCore(
      Stepper_Task,    /* Task function. */
      "Stepper1_Task", /* name of task. */
      10000,           /* Stack size of task */
      NULL,            /* parameter of the task */
      2,               /* priority of the task */
      &StepperTask,
      1); /* Task handle to keep track of created task */
}

/// @brief Main loop
void loop()
{
}
