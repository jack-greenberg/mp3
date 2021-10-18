#define OUTER_RIGHT (A1) // blue
#define INNER_RIGHT (A0) // purple
#define INNER_LEFT (A2) // green
#define OUTER_LEFT (A3) // yellow

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float left = 5.0f * analogRead(OUTER_RIGHT) / 1024.0f;
  float right = 5.0f * analogRead(OUTER_LEFT) / 1024.0f;

  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.println(left-right);
  
  delay(100);
}
