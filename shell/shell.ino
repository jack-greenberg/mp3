#include <Shell.h>
#include <avr/eeprom.h>

int eeprom_kp __attribute__((section(".eeprom"))) = 0;
int eeprom_ki __attribute__((section(".eeprom"))) = 0;
int eeprom_kd __attribute__((section(".eeprom"))) = 0;
int eeprom_speed __attribute__((section(".eeprom"))) = 0;

int kp, ki, kd, speed;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  /*
   * Initialize EEPROM memory.
   * 
   * Fetches previously saved values from nonvolatile EEPROM memory.
   */
  eeprom_read_block(&kp, &eeprom_kp, sizeof(kp));
  eeprom_read_block(&ki, &eeprom_ki, sizeof(ki));
  eeprom_read_block(&kd, &eeprom_kd, sizeof(kd));
  eeprom_read_block(&speed, &eeprom_speed, sizeof(speed));

  Serial.println("\nInitialized tunable parameters");
  Serial.print("  Kp: "); Serial.println(kp);
  Serial.print("  Ki: "); Serial.println(ki);
  Serial.print("  Kd: "); Serial.println(kd);
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

void loop() {
  shell_task();
}


/*
 * Macro to define a function that allow users to set the value of a variable
 */
#define SHELL_SETTER_CMD(var) \
  int cmd_set_##var(int argc, char** argv) { \
    if (argc == 1) { \
      Serial.print(var); \
      return SHELL_RET_SUCCESS; \
    } \
  \
    var = atoi(argv[argc-1]); \
    eeprom_update_block(&var, &eeprom_ ## var, sizeof(var)); \
  \
    Serial.print("Set speed to: "); \
    Serial.print((int)atoi(argv[argc-1])); \
  \
    return SHELL_RET_SUCCESS; \
  }

SHELL_SETTER_CMD(kp)
SHELL_SETTER_CMD(ki)
SHELL_SETTER_CMD(kd)
SHELL_SETTER_CMD(speed)


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
