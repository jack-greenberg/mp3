//#include <Wire.h>
//#include <Adafruit_MotorShield.h>
//#include "utility/Adafruit_MS_PWMServoDriver.h"
//
//Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//Adafruit_DCMotor *right = AFMS.getMotor(1);
//Adafruit_DCMotor *left = AFMS.getMotor(2);

typedef struct {

float kp; //proportional
float ki; //integral
float kd; //derivative

float tau; //derivative low-pass filter time constant

//Output Limits
int lim_min;
int lim_max; 

float T; //sample time

float integrator; 
float prev_error; 
float differentiator; 
float prev_measurement;

float out;

} PID ;

//Initialize values:
PID pid = {
  .kp = 2.0f, 
  .ki = 0.5f,
  .kd = .25f, 
  .tau = .02f,
  .lim_min = 0,
  .lim_max = 100, 
  .T = .01f, 
  .integrator = 0.0f, 
  .prev_error = 0.0f, 
  .differentiator = 0.0f, 
  .prev_measurement = 0.0f,
  .out = 0.0f
};
  

void setup() {
//  AFMS.begin();
  Serial.begin(9600);
}

void loop() {

  float left_ir = 5.0f * (float)analogRead(A4) / 1024.0f;
  
  float right_ir = 5.0f * (float)analogRead(A5) / 1024.0f; 

//  Serial.print(left_ir); Serial.print(", "); Serial.println(right_ir);

  float diff = left_ir - right_ir; 

  float error = 0 - diff; 

  float proportional = pid.kp * error; 

  pid.integrator = pid.integrator + 0.5f * pid.ki * pid.T * (error + pid.prev_error); 

  //anti-windup potentially 

  pid.differentiator = -(2.0f * pid.kd * (diff - pid.prev_measurement)
                       +(2.0f * pid.tau - pid.T)*pid.differentiator)
                       /(2.0f * pid.tau + pid.T);

  pid.out = proportional + pid.integrator + pid.differentiator; 

  if (pid.out > pid.lim_max){
    pid.out = pid.lim_max; 
  }
  else if (pid.out < pid.lim_min){
    pid.out = pid.lim_min; 
  }

  if (error > 0){
    //change right motor speed
  }
  else if (error < 0){
    //change left motor speed
  }

  Serial.println(diff);
  delay(100);
  pid.prev_error = error;
  pid.prev_measurement = diff; 

  
  
}
