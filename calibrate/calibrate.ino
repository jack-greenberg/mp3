#define LEFT_IR (A4)
#define RIGHT_IR (A5)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int left = analogRead(LEFT_IR);
  int right = analogRead(RIGHT_IR);

  Serial.print(left);
  Serial.print(",");
  Serial.println(right);
  
  delay(100);
}
