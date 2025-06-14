#include <Wire.h>

#define SLAVE_ADDR 0x08

void setup() {
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(sendData);
  Serial.begin(9600);

  pinMode(2, INPUT); // IR Right 1 (back wheel)
  pinMode(4, INPUT); // IR Front Right 1 (Right)
  pinMode(3, INPUT); // IR Front Right 2 (Left)
  pinMode(6, INPUT); // IR Front Left 1 (Right)
  pinMode(5, INPUT); // IR Front Left 2 (Left)
  pinMode(7, INPUT); // IR Left 1 (back wheel)
  pinMode(13, INPUT_PULLUP); // Button
}

void loop() {
  delay(100);
}

void sendData() {

  int ir1 = digitalRead(2);  // IR Right 1 (back wheel)
  int ir2 = digitalRead(4);  // Front Right 1 (Right)
  int ir3 = digitalRead(3);  // Front Right 2 (Left)
  int ir4 = digitalRead(6);  // Front Left 1 (Right)
  int ir5 = digitalRead(5);  // Front Left 2 (Left)
  int ir6 = digitalRead(7);  // IR Left 1 (back wheel)
  int button = digitalRead(13); // Button
  int foundBall = 0;
  int pressedButton = 0;

  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float distance = 12.08 * pow(voltage, -1.058);

  if (distance <= 6.0) {
    foundBall = 0;
  }
  else {
    foundBall = 1;
  }

  if (button == HIGH) {
    pressedButton = 0;
  } 
  else {
    pressedButton = 1;
  }

  String value = String(ir1) + String(ir2) + String(ir3) + String(ir4) + String(ir5) + String(ir6) + String(foundBall) + String(button);

  Serial.println("Request received. Sending : " + value);

  Wire.write(value.c_str());
}
