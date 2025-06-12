#include "process_control.h"


void setup() {
  Serial.begin(115200);
  setup_InOut();
  setupMotors();
  servo_setup();
  setupTimer(0, 80, 50);//Initialize timer 0,prescaler = 80, timer tick = 70 =>>Timer interrupt at every 70us
  setupOled();
  setup_RotaryEncoder_SW();

  delay(5000);
  Servo_write(90);
  Serial.println("Start");
}

//bool con = false;

void loop() {
  
  Encoder_SW.tick();
  interface();
  if(process_Enable){
    auto_cut(setQuantity, setLength, setAccuracy, setSpeed);
  }
  else{
    current_stage = 0;
    timer_disable();
    motorControl(0,0);
    motorControl(0,1);
  }
  // currentTime = millis();
  // if(currentTime - previousTime > 5) {
  //   previousTime = currentTime;

  //   curTime_Filter = micros();
  //   float deltaT = ((float)(curTime_Filter-prevTime_Filter))/1.0e6;
  //   prevTime_Filter = curTime_Filter;
    
  //   if(digitalRead(saw_homeSW) and digitalRead(belt_homeSW)){
  //     int PV = motors[1].Pulses*6.6;
  //     motorControl(PID_controller(0, PV, deltaT), 0);
  //     Serial.print(motors[1].Pulses);Serial.print("   ");
  //     Serial.println(motors[0].Pulses);
  //   }
  //   else{
  //     digitalWrite(motors[0].pinA, LOW);
  //     digitalWrite(motors[0].pinB, LOW);
  //   }
  // }

  // currentTime = millis();
  // if(currentTime - previousTime > 10) {
  //   previousTime = currentTime;
    
  //   curTime_Filter = micros();
  //   float deltaT = ((float)(curTime_Filter-prevTime_Filter))/1.0e6;
  //   prevTime_Filter = curTime_Filter;

  //   //motors[0].Filted_Pulse =  0.6395*motors[0].Filted_Pulse + 0.1803*motors[0].Pulses + 0.1803*motors[0].prevPulses;
  //   //motors[1].Filted_Pulse =  0.6395*motors[1].Filted_Pulse + 0.1803*motors[1].Pulses + 0.1803*motors[1].prevPulses;
  //   // motors[0].preFilted_Pulse = motors[0].Filted_Pulse;
  //   // motors[1].preFilted_Pulse = motors[1].Filted_Pulse;
  //   // ledcWrite(motors[1].PWM_chanel, 1023);
  //   // motors[1].direction = true;
  //   // digitalWrite(motors[1].pinA, motors[1].direction);
  //   // digitalWrite(motors[1].pinB, !motors[1].direction);

  //   ledcWrite(motors[1].PWM_chanel, 650);
  //   motors[1].direction = true;
  //   digitalWrite(motors[1].pinA, motors[1].direction);
  //   digitalWrite(motors[1].pinB, !motors[1].direction);

  //   ledcWrite(motors[0].PWM_chanel, 650);
  //   motors[0].direction = true;
  //   digitalWrite(motors[0].pinA, motors[0].direction);
  //   digitalWrite(motors[0].pinB, !motors[0].direction);

  //   motors[0].RPM = (motors[0].Pulses - motors[0].prevPulses)/deltaT *60/600;
  //   motors[1].RPM = (motors[1].Pulses - motors[1].prevPulses)/deltaT *60/600;
  //   motors[0].prevPulses = motors[0].Pulses;
  //   motors[1].prevPulses = motors[1].Pulses;

  //   // if(motors[1].Pulses <= 6*600){
  //   //   ledcWrite(motors[1].PWM_chanel, 600);
  //   //   motors[1].direction = true;
  //   //   digitalWrite(motors[1].pinA, motors[1].direction);
  //   //   digitalWrite(motors[1].pinB, !motors[1].direction);
  //   // }
  //   // else if(!con){
  //   //   ledcWrite(motors[1].PWM_chanel, 600);
  //   //   motors[1].direction = true;
  //   //   digitalWrite(motors[1].pinA, motors[1].direction);
  //   //   digitalWrite(motors[1].pinB, !motors[1].direction);

  //   //   motorControl(PID_controller(0, motors[1].Pulses - 6*600 , deltaT), 0);
  //   //   if((motors[0].Pulses >= motors[1].Pulses - 3599) or (motors[0].Pulses <= motors[1].Pulses - 3601)){
  //   //     if(digitalRead(15) == LOW){
  //   //       con = true;
  //   //     }
  //   //     Serial.println(100);
  //   //   }
  //   // // Serial.print(motors[0].Pulses); Serial.print(" \t");
  //   // // Serial.print(motors[1].Pulses - 6*600); Serial.println();
  //   // }
  //   // else{
  //   //   // digitalWrite(motors[1].pinA, LOW);
  //   //   // digitalWrite(motors[1].pinB, LOW);
  //   //   digitalWrite(motors[0].pinA, LOW);
  //   //   digitalWrite(motors[0].pinB, LOW);
  //   // }
  //   Serial.print(motors[0].RPM); Serial.print(" \t");
  //   Serial.print(motors[1].RPM); Serial.println();
  // }
}
