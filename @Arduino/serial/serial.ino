#include <SoftwareSerial.h>

const int buttonPin = 2;
int buttonState = 0;
int prevButtonState = 0;
int isOn = 0;

void setup() {
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  prevButtonState = digitalRead(buttonPin);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState != prevButtonState && buttonState == LOW) {
    isOn = !isOn;
  }

  if(Serial.available() > 0) {

    int byte = Serial.read();

    if(byte == 8) {
      delay(10);
      Serial.println(isOn);
    }

  }

  prevButtonState = buttonState;
  delay(50);

}
