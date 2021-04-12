#include <Wire.h>
#include <SoftPWM.h>
#include <Servo.h>
#define servopin  11
Servo servo;

int animation_delay = 65;           // The higher the number, the slower the timing.
int ledPins[] = {6, 5, 4, 3, 2};       // an array of pin numbers to which LEDs are attached
int pinCount = 5;           // the number of pins (i.e. the length of the array)
int I2C_RECEIVED_BYTE = 0;

void setup() {
  servo.attach(servopin);
  SoftPWMBegin();
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    SoftPWMSet(ledPins[thisPin], 0);
    SoftPWMSetFadeTime(ledPins[thisPin], 150, 150);
  }
  Wire.begin(8);        // join i2c bus (address optional for master)
  Wire.onReceive(I2C_RECIEVE_EVENT);
  Serial.begin(9600);  // start serial for output
}

void loop() {
  switch (I2C_RECEIVED_BYTE) {
    case 0:
      servo.write(90);
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 255);
        delay(animation_delay);
        SoftPWMSet(ledPins[thisPin], 1);
      }
      for (int thisPin = pinCount - 1; thisPin >= 0; thisPin--) {
        SoftPWMSet(ledPins[thisPin], 255);
        delay(animation_delay);
        SoftPWMSet(ledPins[thisPin], 1);
      }
      break;
    case 1:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
        SoftPWMSetFadeTime(ledPins[thisPin], 20, 20);
      }
      servo.write(90);
      servo.detach();
      while (1) {
        SoftPWMSet(ledPins[0], 50);
        SoftPWMSet(ledPins[1], 50);
        SoftPWMSet(ledPins[2], 50);
        SoftPWMSet(ledPins[3], 50);
        SoftPWMSet(ledPins[4], 50);
        delay (50);
        SoftPWMSet(ledPins[0], 1);
        SoftPWMSet(ledPins[1], 1);
        SoftPWMSet(ledPins[2], 1);
        SoftPWMSet(ledPins[3], 1);
        SoftPWMSet(ledPins[4], 1);
        delay (250);
      }
    case 2:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
      }
      SoftPWMSet(ledPins[0], 255);
      delay(5000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 3:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
      }
      SoftPWMSet(ledPins[0], 255);
      SoftPWMSet(ledPins[1], 255);
      delay(5000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 4:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
      }
      SoftPWMSet(ledPins[0], 255);
      SoftPWMSet(ledPins[1], 255);
      SoftPWMSet(ledPins[2], 255);
      delay(5000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 5:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
      }
      SoftPWMSet(ledPins[0], 255);
      SoftPWMSet(ledPins[1], 255);
      SoftPWMSet(ledPins[2], 255);
      SoftPWMSet(ledPins[3], 255);
      delay(5000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 6:
      for (int thisPin = 0; thisPin < pinCount; thisPin++) {
        SoftPWMSet(ledPins[thisPin], 1);
      }
      SoftPWMSet(ledPins[0], 255);
      SoftPWMSet(ledPins[1], 255);
      SoftPWMSet(ledPins[2], 255);
      SoftPWMSet(ledPins[3], 255);
      SoftPWMSet(ledPins[4], 255);
      delay(5000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 7:
      SERVO_ANIMATION();
      servo.write(180);
      delay (1000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    case 8:
      break;
    case 9:
      SERVO_ANIMATION();
      servo.write(0);
      delay (1000);
      I2C_RECEIVED_BYTE  = 0;
      break;
    default:
      break;
  }
}

void I2C_RECIEVE_EVENT(int bytes) {
  I2C_RECEIVED_BYTE = Wire.read();    // read one character from the I2C
  Serial.println(I2C_RECEIVED_BYTE);
}

void SERVO_ANIMATION() {
  SoftPWMSet(ledPins[0], 1);
  SoftPWMSet(ledPins[1], 1);
  SoftPWMSet(ledPins[2], 1);
  SoftPWMSet(ledPins[3], 1);
  SoftPWMSet(ledPins[4], 1);
}
