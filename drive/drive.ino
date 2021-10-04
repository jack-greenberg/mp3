#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right = AFMS.getMotor(1);
Adafruit_DCMotor *left = AFMS.getMotor(2);

void setup() {
  AFMS.begin();

//  right->setSpeed(150);
//  right->run(FORWARD);
}

void loop() {
  // put your main code here, to run repeatedly:

}
