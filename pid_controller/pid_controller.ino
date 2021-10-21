#include <Shell.h>
#include <avr/eeprom.h>

#include "pid.h"

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define SHELL_SETTER_CMD(name, var) \
  int cmd_set_##name(int argc, char** argv) { \
    if (argc == 1) { \
      return SHELL_RET_SUCCESS; \
    } \
    \
    var = atof(argv[argc-1]); \
    eeprom_update_block(&var, &eeprom_ ## name, sizeof(var)); \
    \
    return SHELL_RET_SUCCESS; \
  }

#define OUTER_RIGHT (A1) // blue
#define INNER_RIGHT (A0) // purple
#define INNER_LEFT (A2) // green
#define OUTER_LEFT (A3) // yellow

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right = AFMS.getMotor(1);
Adafruit_DCMotor *left = AFMS.getMotor(2);

float eeprom_kp __attribute__((section(".eeprom"))) = 0;
float eeprom_ki __attribute__((section(".eeprom"))) = 0;
float eeprom_kd __attribute__((section(".eeprom"))) = 0;
float eeprom_cal __attribute__((section(".eeprom"))) = 0;
int eeprom_speed __attribute__((section(".eeprom"))) = 0;

int speed = 0; // Base speed of motors
float cal = 0.0f; //Sensor setpoint

//Initialize values:
PID pid = { 0 };

/*
   Used for UART shell
*/
int shell_reader(char * data)
{
  // Wrapper for Serial.read() method
  if (Serial.available()) {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

void shell_writer(char data)
{
  // Wrapper for Serial.write() method
  //  Serial.write(data);
}

SHELL_SETTER_CMD(kp, pid.kp)
SHELL_SETTER_CMD(ki, pid.ki)
SHELL_SETTER_CMD(kd, pid.kd)
SHELL_SETTER_CMD(speed, speed)
SHELL_SETTER_CMD(cal, cal)

void setup() {
  AFMS.begin();
  Serial.begin(9600);

  /*
     Initialize EEPROM memory and PID controller

     Fetches previously saved values from nonvolatile EEPROM memory.
  */
  eeprom_read_block(&pid.kp, &eeprom_kp, sizeof(float));
  eeprom_read_block(&pid.ki, &eeprom_ki, sizeof(float));
  eeprom_read_block(&pid.kd, &eeprom_kd, sizeof(float));

  pid.lim_min = -70;
  pid.lim_max = 70;
  pid.T = 0.01f;
  pid.tau = 0.1f;

  //  Serial.println("\nInitialized tunable parameters");
  Serial.print("  Kp: "); Serial.println(pid.kp);
  Serial.print("  Ki: "); Serial.println(pid.ki);
  Serial.print("  Kd: "); Serial.println(pid.kd);

  /*
     Initialize shell for setting params
  */
  shell_init(shell_reader, shell_writer, 0);
  shell_register(cmd_set_kp, PSTR("kp"));
  shell_register(cmd_set_ki, PSTR("ki"));
  shell_register(cmd_set_kd, PSTR("kd"));
  shell_register(cmd_set_speed, PSTR("speed"));
  shell_register(cmd_set_cal, PSTR("cal"));

  //  Serial.println("Error, Left motor speed, Right motor speed");
}

void loop() {
  shell_task();

  /*
    Reading outer sensor values and converting to voltage
  */
  float left_ir = 5.0f * (float)analogRead(OUTER_LEFT) / 1024.0f;
  float right_ir = 5.0f * (float)an      //No hard angle turns:
alogRead(OUTER_RIGHT) / 1024.0f;

  float diff = (left_ir - right_ir);

  float out = pid_step(&pid, cal, diff);

  if (speed > 0) {
    //Special cases where there are hard angle turns:
    if (left_ir - right_ir > 2.5) {
      left->setSpeed(speed);
      right->setSpeed(speed);
      left->run(FORWARD);
      right->run(FORWARD);
    } else if (left_ir - right_ir < -2.5) {
      left->setSpeed(speed);
      right->setSpeed(speed);
      left->run(BACKWARD);
      right->run(BACKWARD);

    } 
    
    //No hard angle turns:
    else {
      Serial.print(cal - diff);
      Serial.print(", ");

      Serial.print(speed + out);
      Serial.print(", ");
      Serial.print(speed - out);

      /*
          Setting motor speed based on PID output
      */
      //Adjusting to the right:
      if (speed + out < 0) {
        left->setSpeed(abs(speed + out));
        left->run(FORWARD);
      } else {
        left->setSpeed(speed + out);
        left->run(BACKWARD);
      }

      //Adjusting to the left:
      if (speed - out < 0) {
        right->setSpeed(abs(speed - out));
        right->run(BACKWARD);
      } else {
        right->setSpeed(speed - out);
        right->run(FORWARD);
      }

      Serial.println("");
    }
  } else {
    // Set left motor speed
    left->setSpeed(0);
    left->run(BACKWARD);

    // Set right motor speed
    right->setSpeed(0);
    right->run(FORWARD);
  }

  delay(10); // PID timestep is 10ms
}
