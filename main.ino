#include <ECE3.h>

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;
const int left_bump0 = 24;
const int left_bump1 = 25;
const int left_bump2 = 6;
const int left_bump3 = 27;
const int left_bump4 = 8;
const int left_bump5 = 28;
uint16_t sensorValuesWhite[5][8];
uint16_t sensorValues[8];
uint16_t sensorValuesave[8];
float fusion;
int base = 70;
float kP = 0.052;
float kD = .745;
float pastError = 0;
float dTime = 30;
bool hasTurned = false;
bool check1 = false;
bool allBlack = true;

void setup() {
  ECE3_Init();
  
  // put your setup code here, to run once:
  pinMode(left_bump0,INPUT_PULLUP);
  pinMode(left_bump1,INPUT_PULLUP);
  pinMode(left_bump2,INPUT_PULLUP);
  pinMode(left_bump3,INPUT_PULLUP);
  pinMode(left_bump4,INPUT_PULLUP);
  pinMode(left_bump5,INPUT_PULLUP);

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  for(int i = 0; i < 5; i++) {
    ECE3_read_IR(sensorValuesWhite[i]);
  }

  sensorValuesave[0] = 829;
  sensorValuesave[1]= 740;
  sensorValuesave[2] = 694;
  sensorValuesave[3]= 648;
  sensorValuesave[4]= 736;
  sensorValuesave[5]=853;
  sensorValuesave[6]=740;
  sensorValuesave[7] = 900;
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  float leftSpd = base;
  float rightSpd = base;
  
  if (digitalRead(left_bump0) == 0 || digitalRead(left_bump1) == 0 || digitalRead(left_bump2) == 0 || digitalRead(left_bump3) == 0 || digitalRead(left_bump4) == 0 || digitalRead(left_bump5) == 0){
    analogWrite(left_pwm_pin,0);
    analogWrite(right_pwm_pin, 0);
    delay(10000000000000000000);
  }
  
  ECE3_read_IR(sensorValues);
  
  for (int i = 0; i < 8; i++){
    if(sensorValues[i] < sensorValuesave[i]){
      sensorValues[i] = 0;  
    } else {
      sensorValues[i] = sensorValues[i] - sensorValuesave[i];
    }
    sensorValues[i] = (sensorValues[i] * 1000.0)/2500;
  }

  if (detectTurnBar()){
    if(hasTurned) {
      ChangeWheelSpeeds(base, 0, base, 0);
      delay(10000000000000000);
    } else if (!check1){
      check1 = true; 
    } else if (check1){
      turnAround();
      hasTurned = true; 
    }
     
  } else {
    check1 = false; 
  }
  
  fusion = (-7.5*sensorValues[0] - 7*sensorValues[1] - 6*sensorValues[2] - 1*sensorValues[3] + 1*sensorValues[4] + 6*sensorValues[5] + 7* sensorValues[6] + 7.5*sensorValues[7])/4; 
  //Serial.println(fusion);
 
  float correction = kP * fusion + kD * (fusion - pastError);


  if (sensorValues[0] >= (sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7]) || 
    sensorValues[7] >= (sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[0] + sensorValues[6])) {
    correction *= 255;
  }
  leftSpd -= correction;
  rightSpd += correction;


  //if correction too high
  if (leftSpd > 255){
   leftSpd = 255;
  }
  
  if (rightSpd > 255){
   rightSpd = 255;    
  }

  //if correction too low
  if (leftSpd < 0){
    leftSpd = 0;
  }
  
  if (rightSpd < 0){
    rightSpd = 0;    
  }

  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);

  pastError = fusion;
}

void turnAround(){
  ChangeWheelSpeeds(base, 0, base, 0);
  
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,HIGH);

  ChangeWheelSpeeds(0, 255, 0, 255);

  delay(dTime); //how long it takes to turn

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(right_dir_pin,LOW);
}

bool detectTurnBar(){
  int num = 0;
  
  for(int i = 0; i < 8; i++){
    num += sensorValues[i];  
  }
  
  if (num > 4000){
    return true;
  }
  
  return false;
}

void ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
  int diffLeft = finalLeftSpd-initialLeftSpd;
  int diffRight = finalRightSpd-initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft)/numSteps; // left in(de)crement
  int deltaRight = (diffRight)/numSteps; // right in(de)crement
  
  for(int k=0; k<numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);
    analogWrite(right_pwm_pin,pwmRightVal);
    delay(30);
  }
  
  analogWrite(left_pwm_pin,finalLeftSpd);
  analogWrite(right_pwm_pin,finalRightSpd);
}


////CALIBRATION DATA
//int sensorMax[8] = {2500, 2500, 2500, 1452.4, 1556.6, 2367.6, 2158.8, 2500};
//int sensorMin[8] = {455.4, 883.2, 741.2, 623.6, 539.8, 647.2, 628.25,  769.25}; 
//uint16_t sensorValues[8];
//float kP = 0.051; //sensitivity factor of wheel when you turn
//float kD = 0.742; //derivative of kP
//
////BUMP SWITCHES
//uint16_t bumpSw0_pin = 24;
//uint16_t bumpSw1_pin = 25;
//uint16_t bumpSw2_pin = 6;
//uint16_t bumpSw3_pin = 27;
//uint16_t bumpSw4_pin = 8;
//uint16_t bumpSw5_pin = 28;
//uint16_t bumpSw0Reading;
//uint16_t bumpSw1Reading;
//uint16_t bumpSw2Reading;
//uint16_t bumpSw3Reading;
//uint16_t bumpSw4Reading;
//uint16_t bumpSw5Reading;
//
////USER SWITCHES (sides of LaunchPad)
//uint16_t userSw1_pin = 73;
//int userSw1Reading;
//uint16_t userSw2_pin = 74;
//int userSw2Reading;
//
////LEFT MOTOR
//const int LEFT_NSLP_PIN = 31;
//const int LEFT_DIR_PIN = 29;
//const int LEFT_PWM_PIN = 40;
//const int BASE_LEFT_SPEED = 30;
//
////RIGHT MOTOR
//const int RIGHT_NSLP_PIN = 11;
//const int RIGHT_DIR_PIN = 30;
//const int RIGHT_PWM_PIN = 39;
//const int BASE_RIGHT_SPEED = 30;
//
////OTHER VARIABLES FOR ACTIVE CAR
//
//
//void setup() {
//// put your setup code here, to run once:
//  ECE3_Init();
//
//  pinMode(LEFT_NSLP_PIN, OUTPUT);
//  pinMode(LEFT_DIR_PIN, OUTPUT);
//  pinMode(LEFT_PWM_PIN, OUTPUT);
//  pinMode(RIGHT_NSLP_PIN, OUTPUT);
//  pinMode(RIGHT_DIR_PIN, OUTPUT);
//  pinMode(RIGHT_PWM_PIN, OUTPUT);
//
//  digitalWrite(LEFT_DIR_PIN,LOW);
//  digitalWrite(LEFT_NSLP_PIN,HIGH);
//  digitalWrite(RIGHT_DIR_PIN,LOW);
//  digitalWrite(RIGHT_NSLP_PIN,HIGH);
//}
//
//void loop() {
//  analogWrite(LEFT_PWM_PIN, BASE_LEFT_SPEED);
//  analogWrite(RIGHT_PWM_PIN, BASE_RIGHT_SPEED);
//}
