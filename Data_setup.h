// ##################################################################################
//                Define the pins that are used
// ##################################################################################
#define sawAxis_pinEncoder  15
#define sawAxis_pinPWM      25
#define sawAxis_pinA        32  //
#define sawAxis_pinB        33  //

#define beltAxis_pinEncoder 13
#define beltAxis_pinPWM     14
#define beltAxis_pinA       26
#define beltAxis_pinB       27

#define oled_SDA  21
#define oled_SCL  22

#define rotary_SW   5
#define rotary_DT   18
#define rotary_CLK  19

#define saw_homeSW    12
#define saw_endSW     4
#define belt_homeSW   35

#define servoPin  23
#define relayPin  2
// ##################################################################################
//                 Define the variables that are used for Motor
// ##################################################################################
const int freq = 10000;
const int resolution = 10;

float Kp[2] = {6, 4};
float Ki[2] = {3, 0.8};
float Kd[2] = {0.01, 0.01};
float preIntegral[2] = {0, 0};
int prev_error[2] = {0, 0};

struct MOTOR {
  bool direction = true;
  int RPM;
  int prevPulses;
  volatile int Pulses;
  volatile bool PulseState;
  volatile bool LastState;
  volatile bool Samp_1;
  volatile bool Samp_2;
  volatile bool Samp_3;
  uint8_t PWM_chanel;
  uint8_t pinA;
  uint8_t pinB;
  uint8_t pinPWM;
  uint8_t pinEncoder_A;
};

MOTOR motors[2];
uint8_t process_stage = 10;
uint8_t belt_home_stage = 0;
uint8_t saw_home_stage = 0;
uint8_t saw_follow_stage = 0;

int setSpeed = 500;
int setLength = 50;
int setDiameter = 16;
int setQuantity = 5;
int setAccuracy = 10;
int curQuantity;
// ##################################################################################
//                Define the variables that are used for timer
// ##################################################################################
hw_timer_t *My_timer = NULL;

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long previousTimeServo = 0;

unsigned long curTime_Filter = 0;
unsigned long prevTime_Filter = 0;
// ##################################################################################
//                 Oled Configuration, define the variables
// ##################################################################################
#include <Wire.h>
#include "SSD1306Wire.h"

SSD1306Wire display(0x3c, oled_SDA, oled_SCL);//SDA_SCL
String Menu_categories[8] = {"Speed", "Length", "Diam", "Num", "Accur", "Start/Stop", "Pause/Resume", "Return"};
String process_stages[6] = {"Stopping", "Homing all", "Running", "Finish","Error", "Paused"};
uint8_t current_stage = 0;
uint8_t nums_Menu_categories = 8;
int Menu = 0;

bool Mode = true;
bool toMenu = false;
bool toDetailMenu = false;
bool process_Enable = false;
bool process_PauseResume = false;
// ##################################################################################
//                Define the variables that are used for SERVO
// ##################################################################################
const int PWMFreq = 50;
const int PWMChannel = 2;
const int PWMResolution = 8;

unsigned long currentTime_Ser = 0;
unsigned long previousTime_Ser = 0;
// ##################################################################################
//                Define the variables that are used for ROTARY ENCODER
// ##################################################################################
#include "OneButton.h"
OneButton Encoder_SW;

bool last_CLK, cur_CLK;