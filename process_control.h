#include "Functions.h"

#define L_1 80  //
#define R  53 //

bool bit_1_1 = false;
int setPulse;

void auto_cut(int quantity, int length, int accuracy, int speed){
  // switch (process_stage){
  //   case 10://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Home all >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //   {
  //     Serial.println("Step: Home all");
  //     bool saw_cond = saw_home_process(-500);
  //     bool belt_cond = belt_home_process(&belt_home_stage);
  //     if(saw_cond && belt_cond){
  //       motors[0].Pulses = 0; //reset pulse
  //       motors[1].Pulses = 0;
  //       process_stage = 15; //Move to next step
  //       belt_home_stage = 0;  //Reset process variable
  //     }
  //   }
  //   break;
  //   case 15://<<<<<<<<<<<<<<<<<<<<<<<<<< Caculate cut point >>>>>>>>>>>>>>>>>>>>>>>>>>
  //     Serial.println("Step: Caculate");
  //     set_pulse = (600.0/R)*(L1 + set_length);
  //     Serial.print("Set pulseeeeeeeeeeeeeeeeeeee: ");Serial.println(set_pulse);
  //     process_stage = 20;
  //   break;
  //   case 20://<<<<<<<<<<<<<<<<<<<<<<<<<<<< Enable motor 1 >>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //     timer_enable();
  //     //belt_enable = true; // Enable motor 1 run;
  //     process_stage = 100; // Move to saw control
  //     Serial.print(motors[0].Pulses); Serial.print(" \t");
  //     Serial.print(motors[1].Pulses); Serial.println();
  //   break;
  //   case 25://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Saw control >>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //   { 
  //     motorControl(set_speed, 1);
  //     currentTime = millis();
  //     if(currentTime - previousTime > 10){
  //       previousTime = currentTime;

  //       curTime_Filter = micros();
  //       float deltaT = ((float)(curTime_Filter-prevTime_Filter))/1.0e6;
  //       prevTime_Filter = curTime_Filter;

  //       if(saw_follow_stage == 0 && motors[1].Pulses >= set_pulse){
  //         motorControl(PID_controller(0, motors[1].Pulses - set_pulse, deltaT), 0);
  //         Serial.println("Start catching");
  //       }
  //       if((motors[0].Pulses >  motors[1].Pulses - set_pulse - 2) and (motors[0].Pulses < motors[1].Pulses - set_pulse + 2)){
  //         Serial.println("Cutting");
  //         if(cutter()){
  //           //CUT done
  //           motors[1].Pulses -= (600.0/R)*(set_length);
  //           saw_follow_stage = 5;
  //           set_quantity --;
  //         }
  //       }
  //       Serial.print(motors[0].Pulses); Serial.print(" \t");
  //       Serial.print(motors[1].Pulses - set_pulse); Serial.println();
  //       Serial.println("Stage 25");
  //     }
  //     if(saw_follow_stage == 5){
  //       bool saw_cond = saw_home_process(-1000);
  //       if(saw_cond)  {
  //         saw_follow_stage = 0;
  //         motors[0].Pulses = 0;
  //         Serial.println("Reset");
  //       }
  //     }
  //     if(set_quantity == 0) process_stage = 30;
  //   }
  //   break;
  //   case 30://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Enable Cut >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //     timer_disable();
  //     process_stage = 35;
  //     Serial.println("stage 35");
  //   break;
  //   case 35://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Home saw >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //     // if (Motors_move_home(1)){
  //     //   if(cutting_numbers == 0){
  //     //     process_stage = 100; //Finish process
  //     //   }
  //     //   else{
  //     //     process_stage = 11; //Move to caculate cut point
  //     //   }
  //     // }
  //     Serial.println("finish");
  //   break;
  //   case 99://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Error >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  //   break;
  //   case 100://<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Finish >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

  //   break;
  // }
  if(curQuantity == quantity) process_stage = 30;
  if(!digitalRead(saw_endSW))  process_stage = 35;
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Home all >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  if(process_stage == 10){
    bool saw_cond = saw_home_process(550);// SAW move home at set speed
    bool belt_cond = belt_home_process(&belt_home_stage, 550);// BELT move home at set speed
    current_stage = 1;
    if(saw_cond && belt_cond){
      process_stage = 15; //Move to next step
      belt_home_stage = 0;  //  Reset process variable for belt home process
    }
  }
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Caculate >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  else if(process_stage == 15){
    setPulse = (length + L_1)*600/R;
    process_stage = 20;
    
    Serial.println("Step: Caculate");
    Serial.print("Set pulseeeeeeeeeeeeeeeeeeee: ");Serial.println(setPulse);
  }
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Enable >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  else if(process_stage == 20){
    timer_enable(); //Enable timer, start read pulse
    process_stage = 25;
    Serial.println("Enable");
    //Reset pulse - eliminate mechanical noise (just  for sure)
    motors[0].Pulses = 0; 
    motors[1].Pulses = 0; 
  }
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Operating >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  else if(process_stage == 25){
    digitalWrite(relayPin,HIGH);
    current_stage = 2;
    Serial.print(motors[0].Pulses); Serial.print(" \t");
    Serial.print(motors[1].Pulses); Serial.print(" \t");
    Serial.print((motors[1].Pulses- setPulse)*6.6);Serial.println();

    motorControl(speed, 1);

    if(motors[1].Pulses < setPulse) {
      motors[0].Pulses = 0; //  Block SAW-pulse, eliminate mechanical noise (just  for sure)
    }
    else if(saw_follow_stage == 0){// Start catching when BELT-pulse reach setpulse
      Serial.println("Start catching");

      int PV = (motors[1].Pulses - setPulse)*6.6;
      currentTime = millis();
      if(currentTime - previousTime > 5){
        previousTime = currentTime;

        curTime_Filter = micros();
        float deltaT = ((float)(curTime_Filter-prevTime_Filter))/1.0e6;
        prevTime_Filter = curTime_Filter;

        motorControl(PID_controller(0, PV, deltaT), 0);
      }

      if((motors[0].Pulses >  PV - accuracy) and (motors[0].Pulses < PV + accuracy)){
        enableCut = true;
        preIntegral[0] = 0;
        prev_error[0] = 0;
      }

      if(enableCut) {
        Serial.println("Cutting");// Start cutting when SAW-pulse reached BELT-pulse
        bool cut_cond = cutter();
        if(cut_cond){// CUT done
          motors[1].Pulses -= (600/R)*(length);
          saw_follow_stage = 5;
          curQuantity ++;
          enableCut = false;
          Serial.println("Cutting  Done");
        }
      }
    }
    if(saw_follow_stage == 5){
      if(process_PauseResume == false){
        current_stage = 2;
        bool saw_cond = saw_home_process(1023);
        if(saw_cond)  {
          Serial.println("Reset");
          saw_follow_stage = 0;
          //  Reset caculate variable
          preIntegral[0] = 0;
          prev_error[0] = 0;
        }
      }
      else{
        current_stage = 5;
        motorControl(0,0);
        motorControl(0,1);
      }
    }
  }
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Finish >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  else if(process_stage == 30){
    curQuantity = 0;
    current_stage = 3;
    timer_disable();
    bool saw_cond = saw_home_process(500);
    bool belt_cond = belt_home_process(&belt_home_stage, 500);
    if(saw_cond && belt_cond){
      process_Enable = false;
      motorControl(0,0);
      motorControl(0,1);
    }
    digitalWrite(relayPin, LOW);
    Serial.println("DONE");
  }
  else if(process_stage == 35){
    current_stage = 4;
    timer_disable();
    motorControl(0,1);
    bool saw_cond = saw_home_process(500);
    if(saw_cond){
      motorControl(0,0);
      Serial.println("STOPING");
      curQuantity = 0;
    }
    digitalWrite(relayPin, LOW);
    Serial.println("ERROR");
  }
}




