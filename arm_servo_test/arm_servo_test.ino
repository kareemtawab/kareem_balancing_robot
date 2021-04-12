#define servopin  11
#include <ServoTimer2.h>
ServoTimer2 servo;

void setup() {
  servo.attach(servopin);
}

void loop()
{
  servo.write(2400);
  delay(2000);
  servo.write(1500);
  delay(2000);
  servo.write(544);
  delay(2000);
  servo.write(1500);
  delay(2000);


  /*
    for (int i = 3000; i > 1500; i -= 10) { // goes from 0 degrees to 180 degrees
    servo.write(i);
    delay(5);
    }

    delay(2000);

    for (int i = 1500; i > 10; i -= 10) { // goes from 0 degrees to 180 degrees
    servo.write(i);
    delay(5);
    }

    delay(2000);

    for (int i = 10; i <= 1500; i += 10) { // goes from 0 degrees to 180 degrees
    servo.write(i);
    delay(5);
    }

    delay(2000);

    for (int i = 1500; i <= 3000; i += 10) { // goes from 0 degrees to 180 degrees
    servo.write(i);
    delay(5);
    }

    delay(2000);
  */
}
