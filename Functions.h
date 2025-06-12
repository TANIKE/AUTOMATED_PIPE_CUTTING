#include "Data_setup.h"

/***************************************************************************************
** Function name:           setup_InOut
** Description:             Initialize Input/Output 
***************************************************************************************/
void setup_InOut(){
  //pinMode(rotary_SW, INPUT);
  pinMode(rotary_DT, INPUT);
  pinMode(rotary_CLK, INPUT);

  pinMode(saw_homeSW, INPUT_PULLUP);
  pinMode(saw_endSW, INPUT_PULLUP);
  pinMode(belt_homeSW, INPUT);
  pinMode(relayPin, OUTPUT);
}

/***************************************************************************************
** Function name:           setupMotors
** Description:             Initialize Input/Output-PWM for Motor
***************************************************************************************/
void setupMotors(void){
  //SAW AXIS
  motors[0].PWM_chanel = 0;
  motors[0].pinEncoder_A = sawAxis_pinEncoder;
  motors[0].pinPWM = sawAxis_pinPWM;
  motors[0].pinA = sawAxis_pinA;
  motors[0].pinB = sawAxis_pinB;
  //BELT AXIS
  motors[1].PWM_chanel = 1;
  motors[1].pinEncoder_A = beltAxis_pinEncoder;
  motors[1].pinPWM = beltAxis_pinPWM;
  motors[1].pinA = beltAxis_pinA;
  motors[1].pinB = beltAxis_pinB;

  pinMode(motors[0].pinEncoder_A, INPUT);
  //pinMode(motors[0].pinPWM, OUTPUT);
  pinMode(motors[0].pinA, OUTPUT);
  pinMode(motors[0].pinB, OUTPUT);

  pinMode(motors[1].pinEncoder_A, INPUT_PULLUP);
  //pinMode(motors[1].pinPWM, OUTPUT);
  pinMode(motors[1].pinA, OUTPUT);
  pinMode(motors[1].pinB, OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(motors[0].pinEncoder_A), countPulse_MOTOR<0>, RISING);
  //attachInterrupt(digitalPinToInterrupt(motors[1].pinEncoder_A), countPulse_MOTOR<1>, RISING);

  ledcSetup(0, freq, resolution);
  ledcAttachPin(motors[0].pinPWM, 0);

  ledcSetup(1, freq, resolution);
  ledcAttachPin(motors[1].pinPWM, 1);
}

/***************************************************************************************
** Function name:           motorControl
** Description:             Control Motor[i] with input PWM value 
***************************************************************************************/
void motorControl(int PWM_val, int motor_i){
  //int out = (int)(abs(Speed)*1023/350);
  int out = abs(PWM_val);
  if (out < 200) out = 300; //Minimum PWM value
  ledcWrite(motors[motor_i].PWM_chanel, out);
  if(PWM_val > 0){
    motors[motor_i].direction = true;
    digitalWrite(motors[motor_i].pinA, motors[motor_i].direction);
    digitalWrite(motors[motor_i].pinB, !motors[motor_i].direction);
  }
  else if(PWM_val < 0){
    motors[motor_i].direction = false;
    digitalWrite(motors[motor_i].pinA, motors[motor_i].direction);
    digitalWrite(motors[motor_i].pinB, !motors[motor_i].direction);
  }
  else {
    digitalWrite(motors[motor_i].pinA, LOW);
    digitalWrite(motors[motor_i].pinB, LOW);
  }
}

/***************************************************************************************
** Function name:           PID_controller
** Description:             Caculate PWM value for Motor[i] to reach desire number os pulses
***************************************************************************************/
int PID_controller(uint8_t motor_i, int setpoint, float Ts){

  float error = setpoint - motors[motor_i].Pulses;
  float Integral = preIntegral[motor_i] + (Ts*(error+prev_error[motor_i])/2);
  int Out = Kp[motor_i]*error + Ki[motor_i]*Integral + Kd[motor_i]*(error-prev_error[motor_i])/Ts;

  if(Out > 1023) Out = 1023;
  if(Out < -1023) Out = -1023;

  preIntegral[motor_i] = Integral;
  prev_error[motor_i] = error;

  return Out;
}

/***************************************************************************************
** Function name:           onTimer
** Description:             ISR function, read Encoders-sampling-count pulses
***************************************************************************************/
void IRAM_ATTR onTimer(){
  for(uint8_t i = 0; i < 2; i++){ //loop through 2 motor
    motors[i].Samp_1 = digitalRead(motors[i].pinEncoder_A); //get current Encoder signal
    if(motors[i].Samp_1 == motors[i].Samp_2 == motors[i].Samp_3) { //if last 3 samples as the same -> enable state
      motors[i].PulseState = motors[i].Samp_1; //Save current State
      if(motors[i].PulseState and !motors[i].LastState){ //Rising edge
        if(motors[i].direction){
          motors[i].Pulses++;
        }
        else{
          motors[i].Pulses--;
        }
      }
      motors[i].LastState = motors[i].PulseState;
    }
    motors[i].Samp_3 = motors[i].Samp_2;
    motors[i].Samp_2 = motors[i].Samp_1;
  }
}

/***************************************************************************************
** Function name:           timer_setup
** Description:             Setup timer
***************************************************************************************/
void setupTimer(uint8_t timerNumber, uint16_t prescaler, uint64_t alarm_value){
  My_timer = timerBegin(timerNumber, prescaler, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, alarm_value, true);
  //timerAlarmEnable(My_timer); //Just Enable
}

/***************************************************************************************
** Function name:           timer_enable
** Description:             Enable timer interrupt
***************************************************************************************/
void timer_enable(){
  timerAlarmEnable(My_timer);
}

/***************************************************************************************
** Function name:           timer_disable
** Description:             Disable timer interrupt
***************************************************************************************/
void timer_disable(){
  timerAlarmDisable(My_timer);
}

/***************************************************************************************
** Function name:           oled_setup
** Description:             Disable timer interrupt
***************************************************************************************/
void setupOled(){
  display.init();
  display.flipScreenVertically();
  display.drawString(10,10,"Connecting to Wifi");
  display.display();
}

/***************************************************************************************
** Function name:           belt_home_process
** Description:             Processing belt axis move to home
***************************************************************************************/
bool belt_home_process(uint8_t *Stage, int16_t PWM_val){
  bool direct = digitalRead(belt_homeSW);
  switch (*Stage){
    case 0: //<<<<<Step 1: Determine direction>>>>>
      if(!direct)  *Stage = 5;
      else  *Stage = 10;
      return false;
    break;
    case 5: // Executted when material under the sensor 
      if(!direct)  motorControl(-PWM_val, 1);
      else *Stage = 15;
      return false;
    break;
    case 10:  // Executted when material beyond the sensor 
      if(direct) motorControl(PWM_val, 1);
      else *Stage = 15;
      return false;
    break;
    case 15: // Belt_home  finish
      digitalWrite(motors[1].pinA, LOW);
      digitalWrite(motors[1].pinB, LOW);
      motors[1].Pulses = 0; // Reset BELT pulse
      //*Stage = 0;// Reset process variable
      return true;
    break;
  }
}

/***************************************************************************************
** Function name:           saw_home_process
** Description:             Processing saw axis move to home
***************************************************************************************/
bool saw_home_process(int16_t PWM_val){
  if(digitalRead(saw_homeSW)){
    //motorControl(-PWM_val, 0);
    ledcWrite(motors[0].PWM_chanel, PWM_val);
    digitalWrite(motors[0].pinA, LOW);
    digitalWrite(motors[0].pinB, HIGH);
    Serial.println("Homing");
    return false;
  }
  else{
    digitalWrite(motors[0].pinA, LOW);
    digitalWrite(motors[0].pinB, LOW);
    motors[0].Pulses = 0;
    Serial.println("Done Homing");
    return true;
  }
}

/***************************************************************************************
** Function name:           servo_setup
** Description:             Setup for timer
***************************************************************************************/
void servo_setup(){
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(servoPin, PWMChannel);
  // ledcWrite(PWMChannel, dutyCycle);
}

/***************************************************************************************
** Function name:           servo_setup
** Description:             Setup for timer
***************************************************************************************/
void Servo_write(uint8_t angle){
  uint8_t duty = map(angle, 0, 180, 5, 32);
  ledcWrite(PWMChannel, duty);
}

/***************************************************************************************
** Function name:           cutter
** Description:             Processing enter the cutter
***************************************************************************************/
uint8_t cut_stage = 0;
uint8_t angle = 90;
bool enableCut = false;
bool cutter(){
  currentTime_Ser = millis();
  if(currentTime_Ser - previousTime_Ser > 30){
    previousTime_Ser = currentTime_Ser;
    if(cut_stage == 0){
    Servo_write(angle);
    angle += 3;
    if(angle >= 140) cut_stage = 2;
    }
    else if(cut_stage == 2){
      Servo_write(90);
      angle -= 3;
      if(angle <= 90) {
        cut_stage = 4;
      }
    }
    else if(cut_stage == 4){
      cut_stage = 0;
      return true;
    }
  }
  return false;
}

/***************************************************************************************
** Function name:           setup_RotaryEncoder_SW
** Description:             Initialize parameter for SW instance
***************************************************************************************/
void setup_RotaryEncoder_SW(){
  Encoder_SW.setup(rotary_SW, INPUT, true); //pin, input type, active low
}

/***************************************************************************************
** Function name:           check_rotary
** Description:             Processing singal from RotaryEncoder
***************************************************************************************/
bool check_rotary(int *count_var,uint8_t step, int min, int max){
  cur_CLK = digitalRead(rotary_CLK);
  if(last_CLK != cur_CLK){
    if(!cur_CLK){
      if(digitalRead(rotary_DT)) *count_var += step;
      else *count_var -= step;

      if(*count_var < min) *count_var = min;
      if(*count_var > max) *count_var = max;

      Serial.println(*count_var);
    }
    last_CLK = cur_CLK;
    return true;
  }
  return false;
}

/***************************************************************************************
** Function name:           display_Main_AutoMode
** Description:             Main interface of Auto mode
***************************************************************************************/
void display_Main_AutoMode(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.setFont(ArialMT_Plain_10);
  display.drawString(2,3,"V: "+String(setSpeed));
  display.drawString(2,13,"N: " + String(setQuantity));
  display.drawRect(-1,-1,38,28);
  display.drawString(47,1,"State: " + process_stages[current_stage]);
  display.drawString(32,32,String(motors[1].Pulses));
  display.drawString(94,32,String(motors[0].Pulses/6.6));
  display.setFont(ArialMT_Plain_16);
  display.drawString(47,12,"Cut: " + String(curQuantity));
  display.drawString(4,29,"FP: ");
  //display.drawString(4,47,"FV: 300");

  display.drawString(66,29,"SP: ");
  display.drawString(66,47,"SS: On");
  display.display();
}

/***************************************************************************************
** Function name:           toAutoMenu
** Description:             switch to Menu interface
***************************************************************************************/
void toAutoMenu(){
  if(!toMenu){
    display.clear();

    display.drawLine(9, 18, 40, 18);
    display.drawLine(9, 46, 40, 46);

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(24,23,Menu_categories[Menu]);
    display.fillRect(54,5, 70, 54);

    display.display();
    toMenu = true;
  }
}

/***************************************************************************************
** Function name:           display_Menu_Auto
** Description:             Menu interface of Auto mode
***************************************************************************************/
void display_Menu_Auto(){
  bool changed = check_rotary(&Menu, 1, 0, nums_Menu_categories - 1);
  if(changed){
    display.clear();

    display.drawLine(9, 18, 40, 18);
    display.drawLine(9, 46, 40, 46);

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.fillRect(54,5, 70, 54);

    if(Menu == 5) {
      if(process_Enable == false) display.drawString(24,23,"Start");
      else display.drawString(24,23,"Stop");
    }
    else if(Menu == 6){
      if(process_Enable == true){
        if(process_PauseResume == true) display.drawString(24,23,"Resume");
        else display.drawString(24,23,"Pause");
      }
      else {
        display.drawString(24,23,Menu_categories[7]);
      }
    }
    else display.drawString(24,23,Menu_categories[Menu]);


    display.display();
  }
}

/***************************************************************************************
** Function name:           chosse_MenuCategory
** Description:             enabale to detail menu
***************************************************************************************/
void chosse_MenuCategory(){
  toDetailMenu = true;
}

/***************************************************************************************
** Function name:           return_Menu
** Description:             disabale to detail menu
***************************************************************************************/
void return_Menu(){
  toDetailMenu = false;

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.display();
}

/***************************************************************************************
** Function name:           set_Speed
** Description:             
***************************************************************************************/
void set_Speed(){
  bool changed = check_rotary(&setSpeed, 2, 300, 1023);

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.setColor(BLACK);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(59, 10, "RPM");

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(89, 32, String(setSpeed));

  display.display();
}

/***************************************************************************************
** Function name:           set_Length
** Description:             
***************************************************************************************/
void set_Length(){
  bool changed = check_rotary(&setLength, 1, 5, 1000);

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.setColor(BLACK);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(59, 10, "mm");

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(89, 32, String(setLength));

  display.display();
}

/***************************************************************************************
** Function name:           set_Diameter
** Description:             
***************************************************************************************/
void set_Diameter(){
  bool changed = check_rotary(&setDiameter, 1, 8, 18);

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.setColor(BLACK);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(59, 10, "mm");

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(89, 32, String(setDiameter));

  display.display();
}

/***************************************************************************************
** Function name:           set_Quantity
** Description:             
***************************************************************************************/
void set_Quantity(){
  bool changed = check_rotary(&setQuantity, 1, 1, 1000);

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.setColor(BLACK);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(59, 10, "Quantity");

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(89, 32, String(setQuantity));

  display.display();
}

/***************************************************************************************
** Function name:           set_Accuracy
** Description:             
***************************************************************************************/
void set_Accuracy(){
  bool changed = check_rotary(&setAccuracy, 1, 10, 50);

  display.setColor(WHITE);
  display.fillRect(54,5, 70, 54);
  display.setColor(BLACK);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(59, 10, "Quantity");

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(89, 32, String(setAccuracy));

  display.display();
}

/***************************************************************************************
** Function name:           interface
** Description:             
***************************************************************************************/
void interface(){
  if(Mode){ // Auto mode
    if(!toMenu){ // Auto main
      display_Main_AutoMode();
      Encoder_SW.attachClick(toAutoMenu);
    }
    else{ // Auto menu
      if(!toDetailMenu){
        display_Menu_Auto();
        Encoder_SW.attachClick(chosse_MenuCategory);
      }
      else{ //detail Menu
        switch(Menu){
          case 0:
            set_Speed();
            Encoder_SW.attachClick(return_Menu);
          break;
          case 1:
            set_Length();
            Encoder_SW.attachClick(return_Menu);
          break;
          case 2:
            set_Diameter();
            Encoder_SW.attachClick(return_Menu);
          break;
          case 3:
            set_Quantity();
            Encoder_SW.attachClick(return_Menu);
          break;
          case 4:
            set_Accuracy();
            Encoder_SW.attachClick(return_Menu);
          break;
          case 5://>>>>>>>Start/Stop process
            process_Enable = !process_Enable;
            Menu = 7;
            process_stage = 10;
          break;
          case 6:
            process_PauseResume = !process_PauseResume;
            Menu = 7;
          break;
          case 7: // Back to main interface
            toMenu = false;
            toDetailMenu = false;
            Menu = 0;
          break;
        }
      }
    }
  }
  else{
  }
}