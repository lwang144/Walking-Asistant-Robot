#include "motor_driver.h"
#include "commands.h"

/* encode */
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
int left_rotate = 0;
int right_rotate = 0;

void initEncoders(){
  pinMode(19, INPUT);
  pinMode(18, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(4, encoderLeftISR, CHANGE);  
  attachInterrupt(5, encoderLeftISR,  CHANGE);  
  attachInterrupt(0, encoderRightISR, CHANGE);
  attachInterrupt(1, encoderRightISR, CHANGE);
}

void encoderLeftISR(){
    if(direction(LEFT) == BACKWARDS){
        left_enc_pos--;
    }else{
        left_enc_pos++;
    }
}

void encoderRightISR(){
    if(direction(RIGHT) == BACKWARDS){
      right_enc_pos--;
    }else{
      right_enc_pos++;
    }
}

  long readEncoder(int i) {
      long encVal = 0L;
  if (i == LEFT)  {
    noInterrupts();
    encVal = left_enc_pos;
    interrupts();
  }
  else {
    noInterrupts();
    encVal = right_enc_pos;
    interrupts();
  }
  return encVal;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      noInterrupts();
      left_enc_pos=0L;
      interrupts();
      return;
    } else { 
      noInterrupts();
      right_enc_pos=0L;
      interrupts();
      return;
    }
  }
//  volatile long left_enc_pos = 0L;
//  volatile long right_enc_pos = 0L;
//  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
//    
//  /* Interrupt routine for LEFT encoder, taking care of actual counting */
//  ISR (PCINT2_vect){
//    static uint8_t enc_last=0;
//        
//  enc_last <<=2; //shift previous state two places
//  enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
//  
//    left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }
//  
//  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
//  ISR (PCINT1_vect){
//        static uint8_t enc_last=0;
//            
//  enc_last <<=2; //shift previous state two places
//  enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
//  
//    right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }
//  
//  /* Wrap the encoder reading function */
//  long readEncoder(int i) {
//    if (i == LEFT) return left_enc_pos;
//    else return right_enc_pos;
//  }
//
//  /* Wrap the encoder reset function */
//  void resetEncoder(int i) {
//    if (i == LEFT){
//      left_enc_pos=0L;
//      return;
//    } else { 
//      right_enc_pos=0L;
//      return;
//    }
//  }


void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}



