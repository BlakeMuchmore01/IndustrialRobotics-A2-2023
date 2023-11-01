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

    // Prints Serial Line of the statis isOn
    if(byte == 1) {
      delay(10);
      Serial.println(isOn);
    }

    // When int 2 is received on serial, isOn is set to 1
    if(byte == 2) {
      delay(10);
      isOn = 1;
      Serial.println(isOn);
    }

    // When int 3 is received on serial, isOn is set to 0
    if(byte == 3) {
      delay(10);
      isOn = 0;
      Serial.println(isOn);
    }

  }

  prevButtonState = buttonState;
  delay(50);

}
