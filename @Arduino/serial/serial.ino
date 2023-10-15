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
    if(isOn == 0) {
      Serial.println("off");
    }
    if(isOn == 1) {
      Serial.println("on");
    }
  }

  prevButtonState = buttonState;
  delay(50);

}
