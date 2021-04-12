#include <SPI.h>   // Comes with Arduino IDE
#include "OneButton.h"
#include "RF24.h"  // Download and Install (See above)
#include <Adafruit_NeoPixel.h>
#include <avr/sleep.h>
#define LEDPIN 6
#define WAKEUPPIN 0

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
RF24 myRadio (9, 10); // "myRadio" is the identifier you will use in following methods

byte addresses[][6] = {"1Node", "2Node"};
int send_byte;  // Data that will be Transmitted from the transmitter
int Rx;
int Ry;
int SW;
unsigned long lastUpdate = 0 ; // for millis() when last update occoured
int received_byte;
unsigned long previousMillis = 0;        // will store last time LED was updated
long interval = 10;           // interval at which to blink (milliseconds)
bool battlowflag;
int i = 0;

OneButton button(A3, true);

void setup() {
  Serial.begin(115200);
  Serial.println(F("NRF24L01 Remote / Balancing Robot"));
  pinMode(WAKEUPPIN, INPUT_PULLUP);
  myRadio.begin();  // Start up the physical nRF24L01 Radio
  myRadio.setChannel(120);  // Above most Wifi Channels
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  myRadio.setPALevel(RF24_PA_LOW);
  //myRadio.setPALevel(RF24_PA_MAX);  // Uncomment for more power
  //myRadio.setDataRate(RF24_2MBPS);
  myRadio.setDataRate(RF24_250KBPS);
  myRadio.setRetries(1, 15);
  myRadio.openWritingPipe( addresses[0]); // 1Node
  myRadio.openReadingPipe(1, addresses[1]); // 2Node
  button.attachClick(clicker);
  button.attachLongPressStart(longpress);
  button.setClickTicks(100);
  button.setPressTicks(1000);
  delay(100);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  myRadio.startListening();
}

void loop() {
  button.tick();
  if (millis() > 5 * 60 * 1000) {
    strip.setBrightness(25);
    strip.setPixelColor(0, strip.Color(0 , 255, 0));
    strip.show();
    myRadio.powerDown();
    sleep_enable();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_cpu();
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval && battlowflag == true) {
    previousMillis = currentMillis;
    interval = 250;
    strip.setBrightness(255);
    if (i == 255) {
      i = 0;
    }
    else {
      i = 255;
    }
    strip.setPixelColor(0, strip.Color(i , 0, 0));
    strip.show();
  }
  else if (currentMillis - previousMillis >= interval && battlowflag == false) {
    previousMillis = currentMillis;
    interval = 10;
    rainbowCycle();
  }
  if (myRadio.available()) // Check for incoming data from transmitter
  {
    while (myRadio.available())  // While there is data ready
    {
      myRadio.read( &received_byte, sizeof(received_byte) );
      Serial.print("RX ");
      Serial.println(received_byte, BIN);
      if (received_byte == 1) {
        battlowflag = true;
      }
    }
  }
  Rx = map(analogRead(A7), 0, 1023, 0, 255);
  Ry = map(analogRead(A6), 0, 1023, 255, 0);

  switch (SW) {
    case 1:
      send_byte |= B00100000;
      break;
    case 2:
      send_byte |= B00010000;
      break;
  }
  if (Rx < 50) {
    send_byte |= B00000001;                //If the variable received_data[0] is smaller then 80 set bit 0 of the send byte variable
  }
  if (Rx > 200) {
    send_byte |= B00000010;                //If the variable received_data[0] is larger then 170 set bit 1 of the send byte variable
  }
  if (Ry < 50) {
    send_byte |= B00001000;                //If the variable received_data[1] is smaller then 80 set bit 3 of the send byte variable
  }
  if (Ry > 200) {
    send_byte |= B00000100;                //If the variable received_data[1] is larger then 170 set bit 2 of the send byte variable
  }
  if (send_byte == B00000000) {                                     //Send the send_byte variable if it's value is larger then 0
    myRadio.startListening();
  }
  else {
    Serial.print("TX ");
    Serial.println(send_byte, BIN);
    myRadio.stopListening();                                    // First, stop listening so we can talk.
    myRadio.write( &send_byte, sizeof(send_byte) );    //Transmit the data
    send_byte = B00000000;
  }
  delay(25);
}

void clicker() {
  SW = 1;
}

void longpress() {
  SW = 2;
}

void rainbowCycle() { // modified from Adafruit example to make it a state machine
  static uint16_t j = 0;
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setBrightness(25);
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
  }
  strip.show();
  j++;
  if (j >= 256 * 5) j = 0;
  lastUpdate = millis(); // time for next change to the display
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
