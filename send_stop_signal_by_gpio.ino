void setup() {
  Serial.begin(9600);

  pinMode(2, OUTPUT); // Analog Distance Sensor Output (PIN 2 TO GPIO 22)

  pinMode(9, INPUT); // IR Right Input

  pinMode(10, INPUT); // IR Front Right Input
  pinMode(3, OUTPUT); // IR Front Right Output (PIN 3 TO GPIO 7)

  pinMode(11, INPUT); // IR Front Left Input
  pinMode(4, OUTPUT); // IR Front Left Output (PIN 4 TO GPIO 8)

  pinMode(12, INPUT); // IR Left Input

  pinMode(5, OUTPUT); // IR Right, Left Output (PIN 5 TO GPIO 24)
}

void loop() {

  int ir_front_right = digitalRead(10);
  int ir_front_left = digitalRead(11);

  int ir_left = digitalRead(12);
  int ir_right = digitalRead(9);

  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 12.08 * pow(voltage, -1.058);

  Serial.println(distance);

  if (distance <= 6.0) {
    digitalWrite(2, HIGH);
    Serial.println("FOUND BALL!");
  }
  else {
    digitalWrite(2, LOW);
  }

  if (ir_front_right == 1) {
    digitalWrite(3, HIGH);
    Serial.println("FOUND FRONT RIGHT!");
  }
  else {
    digitalWrite(3, LOW);
  }

  if (ir_front_left == 1) {
    digitalWrite(4, HIGH);
    Serial.println("FOUND FRONT LEFT!");
  }
  else {
    digitalWrite(4, LOW);
  }

  if (ir_left == 1) {
    digitalWrite(5, HIGH);
    Serial.println("FOUND LEFT!");
  }
  else {
    digitalWrite(5, LOW);
  }

  if (ir_right == 1) {
    digitalWrite(5, HIGH);
    Serial.println("FOUND RIGHT!");
  }
  else {
    digitalWrite(5, LOW);
  }

  delay(50);
}
