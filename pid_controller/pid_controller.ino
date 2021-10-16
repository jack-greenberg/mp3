#include <Shell.h>
#include <avr/eeprom.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define SHELL_SETTER_CMD(name, var) \
  int cmd_set_##name(int argc, char** argv) { \
    if (argc == 1) { \
      Serial.print(var); \
      return SHELL_RET_SUCCESS; \
    } \
  \
    var = atof(argv[argc-1]); \
    eeprom_update_block(&var, &eeprom_ ## name, sizeof(var)); \
  \
    Serial.print("Set speed to: "); \
    Serial.print(var); \
  \
    return SHELL_RET_SUCCESS; \
  }

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right = AFMS.getMotor(1);
Adafruit_DCMotor *left = AFMS.getMotor(2);

int eeprom_kp __attribute__((section(".eeprom"))) = 0;
int eeprom_ki __attribute__((section(".eeprom"))) = 0;
int eeprom_kd __attribute__((section(".eeprom"))) = 0;
int eeprom_speed __attribute__((section(".eeprom"))) = 0;

int speed = 0; // Set-speed of the motor

typedef struct {
  float kp; // proportional
  float ki; // integral
  float kd; // derivative
  
  float tau; // derivative low-pass filter time constant
  
  // Output Limits
  int lim_min;
  int lim_max;
  
  float T; // Sample time
  
  float integrator; 
  float prev_error; 
  float differentiator; 
  float prev_measurement;
  
  float out;

} PID;

//Initialize values:
PID pid = {
  .kp = 0.0f, 
  .ki = 0.0f,
  .kd = 0.0f, 
  .tau = .02f,
  .lim_min = -50,
  .lim_max = 50, 
  .T = .01f,
  .integrator = 0.0f, 
  .prev_error = 0.0f, 
  .differentiator = 0.0f, 
  .prev_measurement = 0.0f,
  .out = 0.0f
};

SHELL_SETTER_CMD(kp, pid.kp)
SHELL_SETTER_CMD(ki, pid.ki)
SHELL_SETTER_CMD(kd, pid.kd)
SHELL_SETTER_CMD(speed, speed)

void setup() {
  AFMS.begin();
  Serial.begin(9600);

  /*
   * Initialize EEPROM memory.
   * 
   * Fetches previously saved values from nonvolatile EEPROM memory.
   */
  eeprom_read_block(&pid.kp, &eeprom_kp, sizeof(float));
  eeprom_read_block(&pid.ki, &eeprom_ki, sizeof(float));
  eeprom_read_block(&pid.kd, &eeprom_kd, sizeof(float));
  eeprom_read_block(&speed, &eeprom_speed, sizeof(speed));

  Serial.println("\nInitialized tunable parameters");
  Serial.print("  Kp: "); Serial.println(pid.kp);
  Serial.print("  Ki: "); Serial.println(pid.ki);
  Serial.print("  Kd: "); Serial.println(pid.kd);
  Serial.print("  Speed: "); Serial.println(speed);

  /*
   * Initialize shell for setting params
   */
  shell_init(shell_reader, shell_writer, 0);
  shell_register(cmd_set_kp, PSTR("kp"));
  shell_register(cmd_set_ki, PSTR("ki"));
  shell_register(cmd_set_kd, PSTR("kd"));
  shell_register(cmd_set_speed, PSTR("speed"));
}

float pid_step(float diff) {
  float error = 0 - diff; // Setpoint is 0 

  // Proportional term
  float proportional = pid.kp * error; 

  // Integrator term
  pid.integrator = pid.integrator + 0.5f * pid.ki * pid.T * (error + pid.prev_error); 

  //anti-windup potentially 

  // Differentiator
  pid.differentiator = -(2.0f * pid.kd * (diff - pid.prev_measurement)
                       +(2.0f * pid.tau - pid.T)*pid.differentiator)
                       /(2.0f * pid.tau + pid.T);

  pid.out = proportional + pid.integrator + pid.differentiator; 

  // Output clamping
  if (pid.out > pid.lim_max){
    pid.out = pid.lim_max; 
  }
  else if (pid.out < pid.lim_min){
    pid.out = pid.lim_min; 
  }

  pid.prev_error = error;
  pid.prev_measurement = diff; 
}

void loop() {
  shell_task();

  float left_ir = 5.0f * (float)analogRead(A4) / 1024.0f;
  float right_ir = 5.0f * (float)analogRead(A5) / 1024.0f;

  float diff = left_ir - right_ir; 
  float out = pid_step(diff);

  // Set left motor speed
  left->setSpeed(speed + out);
  left->run(BACKWARD);

  // Set right motor speed
  right->setSpeed(speed - out);
  right->run(FORWARD);

  delay(10);
}

/*
 * Used for UART shell
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
  Serial.write(data);
}
