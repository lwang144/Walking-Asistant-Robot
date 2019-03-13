#include "commands.h"

// motor one
int enA = 5;
int in1 = 6;
int in2 = 7;
// motor two
int enB = 8;
int in3 = 9;
int in4 = 10;

boolean directionLeft = false;
boolean directionRight = false;

boolean direction(int i){
   if(i == LEFT){
      return directionLeft;
   }else{
      return directionRight;
   }
}
  void initMotorController() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  }

  void setMotorSpeed(int i, int spd) {
    if(spd>MAX_PWM){
      spd=MAX_PWM;
    }
       if(spd<-MAX_PWM){
      spd=-1*MAX_PWM;
    }
    if (i == LEFT){
        if(spd>=0){
            directionLeft = FORWARDS;
            digitalWrite(in2, HIGH);
            digitalWrite(in1, LOW);
            analogWrite(enA, spd);
        }else if(spd < 0){
            directionLeft = BACKWARDS;
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(enA, -spd);
        }
    }
    else {
        if(spd>=0){
            directionRight = FORWARDS;
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(enB, spd);
        }else if(spd<0){
            directionRight = BACKWARDS;
            digitalWrite(in4, HIGH);
            digitalWrite(in3, LOW);
            analogWrite(enB, -spd);
        }
    }
  }


//// motor one
//int enA = 5;
//int in1 = 6;
//int in2 = 7;
//// motor two
//int enB = 11;
//int in3 = 9;
//int in4 = 10;
//
// void initMotorController(){
//
//  // set all the motor control pins to outputs
//  pinMode(enA, OUTPUT);
//  pinMode(enB, OUTPUT);
//  pinMode(in1, OUTPUT);
//  pinMode(in2, OUTPUT);
//  pinMode(in3, OUTPUT);
//  pinMode(in4, OUTPUT);
//
//}
//
//  void setMotorSpeed(int i, int spd) {
//    if(spd>MAX_PWM){
//      spd=MAX_PWM;
//    }
//       if(spd<-MAX_PWM){
//      spd=-1*MAX_PWM;
//    }
//    if (i == LEFT){
//        if(spd>=0){
//
//            digitalWrite(in2, HIGH);
//            digitalWrite(in1, LOW);
//            analogWrite(enA, spd);
//        }else if(spd < 0){
//
//            digitalWrite(in1, HIGH);
//            digitalWrite(in2, LOW);
//            analogWrite(enA, -spd);
//        }
//    }
//    else {
//        if(spd>=0){
//
//            digitalWrite(in3, HIGH);
//            digitalWrite(in4, LOW);
//            analogWrite(enB, spd);
//        }else if(spd<0){
//
//            digitalWrite(in4, HIGH);
//            digitalWrite(in3, LOW);
//            analogWrite(enB, -spd);
//        }
//    }
//  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }


