#ifndef CONFIG_H
#define CONFIG_H

#define MOTOR1_PIN1 D1
#define MOTOR1_PIN2 D2
#define MOTOR2_PIN1 D3
#define MOTOR2_PIN2 D4
#define MOTOR3_PIN1 D5
#define MOTOR3_PIN2 D8

#define MOTOR1_RPM_PIN D7
#define MOTOR2_RPM_PIN D6
#define MOTOR3_RPM_PIN RX // use also TX pin with pinmode funcion_3
// https://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations

#define ENCODER_N 16

float M11 = -0.33;
float M12 = 0.58;
float M13 = 0.33;
float M21 = -0.33;
float M22 = -0.58;
float M23 = 0.33;
float M31 = 0.67;
float M32 = 0;
float M33 = 0.33;

// array of new PWM values [newVal, oldVal]
// int PWM_Const[170][3] = {{91, 90}, {93, 91}, {95, 92}, {98, 93}, {100, 94}, {103, 95}, {105, 96}, {107, 97}, {110, 98}, {112, 99}, {114, 100}, {116, 101}, {118, 102}, {120, 103}, {122, 104}, {124, 105}, {126, 106}, {128, 107}, {129, 108}, {131, 109}, {132, 110}, {134, 111}, {135, 112}, {137, 113}, {138, 114}, {140, 115}, {141, 116}, {142, 117}, {144, 118}, {145, 119}, {146, 120}, {147, 121}, {149, 122}, {150, 123}, {151, 124}, {152, 125}, {153, 126}, {155, 127}, {156, 128}, {157, 129}, {158, 130}, {159, 131}, {160, 132}, {161, 133}, {162, 134}, {163, 135}, {164, 136}, {165, 137}, {166, 138}, {167, 139}, {168, 140}, {169, 141}, {169, 142}, {170, 143}, {171, 144}, {172, 145}, {173, 146}, {173, 147}, {174, 148}, {175, 149}, {176, 150}, {176, 151}, {177, 152}, {178, 153}, {179, 154}, {179, 155}, {180, 156}, {181, 157}, {181, 158}, {182, 159}, {183, 160}, {184, 161}, {184, 162}, {185, 163}, {186, 164}, {187, 165}, {187, 166}, {188, 167}, {189, 168}, {190, 169}, {190, 170}, {191, 171}, {192, 172}, {193, 173}, {194, 174}, {194, 175}, {195, 176}, {196, 177}, {197, 178}, {198, 179}, {199, 180}, {200, 181}, {200, 182}, {201, 183}, {202, 184}, {203, 185}, {204, 186}, {205, 187}, {205, 188}, {206, 189}, {207, 190}, {208, 191}, {208, 192}, {209, 193}, {210, 194}, {211, 195}, {211, 196}, {212, 197}, {213, 198}, {213, 199}, {214, 200}, {215, 201}, {215, 202}, {216, 203}, {216, 204}, {217, 205}, {218, 206}, {218, 207}, {219, 208}, {219, 209}, {220, 210}, {220, 211}, {221, 212}, {221, 213}, {222, 214}, {222, 215}, {223, 216}, {223, 217}, {224, 218}, {225, 219}, {225, 220}, {226, 221}, {226, 222}, {227, 223}, {228, 224}, {228, 225}, {229, 226}, {230, 227}, {231, 228}, {232, 229}, {232, 230}, {233, 231}, {234, 232}, {235, 233}, {236, 234}, {237, 235}, {238, 236}, {239, 237}, {240, 238}, {240, 239}, {241, 240}, {242, 241}, {243, 242}, {244, 243}, {245, 244}, {246, 245}, {246, 246}, {247, 247}, {248, 248}, {249, 249}, {250, 250}, {251, 251}, {252, 252}, {253, 253}, {254, 254}, {255, 255}};
int PWM_Const_2[256] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90, 93, 95, 98, 100, 103, 105, 107, 110, 112, 114, 116, 118, 120, 122, 124, 126, 128, 129, 131, 132, 134, 135, 137, 138, 140, 141, 142, 144, 145, 146, 147, 149, 150, 151, 152, 153, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 169, 170, 171, 172, 173, 173, 174, 175, 176, 176, 177, 178, 179, 179, 180, 181, 181, 182, 183, 184, 184, 185, 186, 187, 187, 188, 189, 190, 190, 191, 192, 193, 194, 194, 195, 196, 197, 198, 199, 200, 200, 201, 202, 203, 204, 205, 205, 206, 207, 208, 208, 209, 210, 211, 211, 212, 213, 213, 214, 215, 215, 216, 216, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 225, 225, 226, 226, 227, 228, 228, 229, 230, 231, 232, 232, 233, 234, 235, 236, 237, 238, 239, 240, 240, 241, 242, 243, 244, 245, 246, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255};

#define PWM_THRESHOLD 85

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