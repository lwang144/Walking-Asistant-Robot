#define BAUDRATE     57600
#define MAX_PWM        255

#include "Arduino.h"
#include "commands.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
#define PID_RATE           30     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[32];
char argv2[32];
long arg1;
long arg2;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[8];
  arg1 = atoi(argv1);  //atoi (表示 ascii to integer)是把字符串转换成整型数的一个函数
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case READ_PIDIN:
      Serial.print( readPidIn(LEFT));
      Serial.print(" ");
      Serial.println( readPidIn(RIGHT));
      break;      
    case READ_PIDOUT:
      Serial.print( readPidOut(LEFT));
      Serial.print(" ");
      Serial.println( readPidOut(RIGHT));
      break;
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = -arg2;     //使一个轮的编码器命令为负，确保走直线时两个轮的方向一致
      Serial.println(arg1);
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {    //strtok_r分解字符串为一组字符串。
        pid_args[i] = atoi(str);
        i++;
      }
//      Kp = pid_args[0];
//      Kd = pid_args[1];
//      Ki = pid_args[2];
//      Ko = pid_args[3];

      left_Kp = pid_args[0];
      left_Kd = pid_args[1];
      left_Ki = pid_args[2];
      left_Ko = pid_args[3];

      right_Kp = pid_args[4];
      right_Kd = pid_args[5];
      right_Ki = pid_args[6];
      right_Ko = pid_args[7];
      Serial.println("OK");
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

unsigned long time = 0, old_time = 0;
void setup() {
  Serial.begin(BAUDRATE);
      //set as inputs
//    DDRD &= ~(1<<LEFT_ENC_PIN_A);
//    DDRD &= ~(1<<LEFT_ENC_PIN_B);
//    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
//    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
//    
//    //enable pull up resistors
//    PORTD |= (1<<LEFT_ENC_PIN_A);
//    PORTD |= (1<<LEFT_ENC_PIN_B);
//    PORTC |= (1<<RIGHT_ENC_PIN_A);
//    PORTC |= (1<<RIGHT_ENC_PIN_B);
//    
//    // tell pin change mask to listen to left encoder pins
//    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
//    // tell pin change mask to listen to right encoder pins
//    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
//    
//    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
//    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  initEncoders();
  initMotorController();
  resetPID();
}

void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();
   // Serial.println(chr);
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    ;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}


