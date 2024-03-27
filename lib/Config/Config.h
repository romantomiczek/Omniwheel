#ifndef CONFIG_H
#define CONFIG_H

#define MOTOR1_PIN1 D1
#define MOTOR1_PIN2 D2
#define MOTOR2_PIN1 D3
#define MOTOR2_PIN2 D4
#define MOTOR3_PIN1 D5
#define MOTOR3_PIN2 D6

float M11 = -0.33;
float M12 = 0.58;
float M13 = 0.33;
float M21 = -0.33;
float M22 = -0.58;
float M23 = 0.33;
float M31 = 0.67;
float M32 = 0;
float M33 = 0.33;

#define PWM_THRESHOLD 150

#endif

/*void calc_speed(float x_dot, float y_dot, float theta_dot)
{
  // calculates motor speeds for 3 wheel omni drive robot
  // see derivation of equations here: https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/
  float PWM1 = -R * theta_dot + x_dot;
  float PWM2 = -R * theta_dot - 0.5 * x_dot - sin(PI / 3.0) * y_dot;
  float PWM3 = -R * theta_dot - 0.5 * x_dot + sin(PI / 3.0) * y_dot;

  int Dir1 = (PWM1 > 0) - (PWM1 < 0); // returns -1 or 1
  int Dir2 = (PWM2 > 0) - (PWM2 < 0);
  int Dir3 = (PWM3 > 0) - (PWM3 < 0);

  PWM1 = (int)abs(PWM1 * scaling_factor); // scale in PWM range of [0, 255]
  PWM2 = (int)abs(PWM2 * scaling_factor);
  PWM3 = (int)abs(PWM3 * scaling_factor);

  M1.spin(Dir1, PWM1);
  M2.spin(Dir2, PWM2);
  M3.spin(Dir3, PWM3);
}*/