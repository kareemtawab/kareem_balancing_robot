///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include <SPI.h>
#include "RF24.h"
#include <Servo.h>
#include "Arduino.h"
#include "PCF8574.h"

PCF8574 pcf8574(0x3F);
float battvolt;
int lowbatttimer;

// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (9, 10); // "myRadio" is the identifier you will use in following methods
byte addresses[][6] = {"1Node"}; // Create address for 1 pipe.
Servo myServo;  // create servo object to control a servo

int gyro_address = 0x68;                                    //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = 685;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 1.0;                                      //Gain setting for the P-controller                          (9)       (14)
float pid_i_gain = 0.02;                                       //Gain setting for the I-controller                       (0.5)     (1)
float pid_d_gain = 0.02;                                    //Gain setting for the D-controller                            (7)       (0.05)
float turning_speed = 35;                                   //Turning speed (20)
float max_target_speed = 100;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start;
int received_byte;
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
int timer_aux;
int batt_aux;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

#define DRIVERS_ENABLEPIN 1
#define SERVO_PIN 6
#define SPEAKER_PIN 7
#define LASER_PIN 8

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Serial.begin(115200);                                                     //Start the serial port at 115200 kbps

  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);
  pcf8574.pinMode(P4, OUTPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT);
  pcf8574.begin();
  pcf8574.digitalWrite(P0, HIGH);
  pcf8574.digitalWrite(P1, HIGH);
  pcf8574.digitalWrite(P2, HIGH);
  pcf8574.digitalWrite(P4, HIGH);
  pcf8574.digitalWrite(P5, HIGH);

  Wire.begin();                                                             //Start the I2C bus as master

  myServo.attach(SERVO_PIN);                                                        // attaches the servo on pin 6 to the servo object
  myServo.write(90);

  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  pinMode(1, OUTPUT);                                                       //Configure digital poort 1 as output
  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(4, OUTPUT);                                                       //Configure digital poort 4 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(SPEAKER_PIN, OUTPUT);                                             //Configure digital poort 7 as output
  pinMode(LASER_PIN, OUTPUT);                                               //Configure digital poort 8 as output
  digitalWrite(DRIVERS_ENABLEPIN, HIGH);
  //digitalWrite(DRIVERS_ENABLEPIN, LOW);

  digitalWrite(LASER_PIN, LOW);

  for (receive_counter = 0; receive_counter < 500; receive_counter++) {     //Create 500 loops
    if (receive_counter % 15 == 0)digitalWrite(LASER_PIN, !digitalRead(LASER_PIN));         //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();           //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();         //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 1000;                                             //Set the loop_timer variable at the next end loop time (4ms)

  myRadio.begin();                                                          //Start up the physical nRF24L01 Radio
  myRadio.setChannel(120);                                                  //Above most Wifi Channels
  //Set the PA Level low to prevent power supply related issues since this is a
  //getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  myRadio.setPALevel(RF24_PA_LOW);
  //myRadio.setPALevel(RF24_PA_MAX);                                          //Uncomment for more power
  //myRadio.setDataRate(RF24_2MBPS);
  myRadio.setDataRate(RF24_250KBPS);
  myRadio.openReadingPipe(1, addresses[0]); // Use the first entry in array 'addresses' (Only 1 right now)
  myRadio.startListening();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if ( myRadio.available()) // Check for incoming data from transmitter
  {
    while (myRadio.available())  // While there is data ready
    {
      myRadio.read( &received_byte, sizeof(received_byte) );
      digitalWrite(LASER_PIN, HIGH);
      receive_counter = 0;                                                   //Reset the receive_counter variable
    }
    Serial.println(received_byte, BIN);
  }
  if (receive_counter <= 100)receive_counter ++;                             //The received byte will be valid for 100 program loops (100 milliseconds)
  else {
    received_byte = 0x00;                                                    //After 100 milliseconds the received byte is deleted
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(SPEAKER_PIN, LOW);
  }

  if (timer_aux <= 300) {                                                    //Increment counter timer_aux if not exceeded 300 program loops
    timer_aux++;
  }
  else {
    myServo.write(90);
    timer_aux = 0;
  }

  if (batt_aux <= 600) {                                                    //Increment counter timer_aux if not exceeded 300 program loops
    batt_aux++;
  }
  else {
    pcf8574.digitalWrite(P0, HIGH);
    pcf8574.digitalWrite(P1, HIGH);
    pcf8574.digitalWrite(P2, HIGH);
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
    batt_aux = 0;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();                  //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer

  if (start == 0 && angle_acc > -10 && angle_acc < 10) {                      //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
    digitalWrite(DRIVERS_ENABLEPIN, LOW);
  }

  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                       //Combine the two bytes to make one integer
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                     //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  angle_gyro -= gyro_yaw_data_raw * +0.0000002;                             //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.175 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if (pid_i_mem > 400)pid_i_mem = 400;                                      //Limit the I-controller to the maximum controller output
  else if (pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > 400)pid_output = 400;                                    //Limit the PI-controller to the maximum controller output
  else if (pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if (pid_output < 5 && pid_output > -5)pid_output = 0;                     //Create a dead-band to stop the motors when the robot is balanced

  if (angle_acc > 70 || angle_acc < -70 || start == 0) {                  //If the robot tips over or the start variable is zero
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
    digitalWrite(DRIVERS_ENABLEPIN, HIGH);
  }

  /*if (angle_gyro > 70 || angle_gyro < -70) {
    digitalWrite(7, HIGH);
    }
    else {
    digitalWrite(7, LOW);
    }*/

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if (received_byte & B00000010) {                                          //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if (received_byte & B00000001) {                                          //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }
  if (received_byte & B00000100) {                                          //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint > -2.5)pid_setpoint -= 0.3;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if (pid_output >= max_target_speed * -1)pid_setpoint -= 0.03;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if (received_byte & B00001000) {                                          //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint < 2.5)pid_setpoint += 0.3;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if (pid_output <= max_target_speed)pid_setpoint += 0.03;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }
  if (received_byte & B00100000) {
    if (angle_gyro < 15) {
      myServo.write(0);
    }
    else {
      myServo.write(180);
    }
  }

  if (received_byte & B00010000 || analogRead(A0) > 1000) {
    drawbattvolt();
    digitalWrite(SPEAKER_PIN, HIGH);
  }

  if (!(received_byte & B00001100)) {                                       //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if (pid_setpoint > 0.5)pid_setpoint -= 0.15;                             //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (pid_setpoint < -0.5)pid_setpoint += 0.15;                       //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0) {                                                  //If the setpoint is zero degrees
    if (pid_output < 0)self_balance_pid_setpoint += 0.02;                    //Increase the self_balance_pid_setpoint if the robot is still moving forewards (0.12)
    if (pid_output > 0)self_balance_pid_setpoint -= 0.02;                    //Decrease the self_balance_pid_setpoint if the robot is still moving backwards (0.12)
    /*if (pid_output <= 0.1 && pid_output >= -0.1) {
      pid_output = 0;
      }*/
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if (pid_output_left > 0)pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
  else if (pid_output_left < 0)pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

  if (pid_output_right > 0)pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
  else if (pid_output_right < 0)pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if (pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if (pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if (pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if (pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while (loop_timer > micros());
  loop_timer += 1000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) {
  //Left motor pulse calculations
  throttle_counter_left_motor = throttle_counter_left_motor + 1;                                         //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttle_counter_left_motor > throttle_left_motor_memory) {           //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if (throttle_left_motor_memory < 0) {                                   //If the throttle_left_motor_memory is negative
      PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if (throttle_counter_left_motor == 1)PORTD |= 0b00000100;            //Set output 2 high to create a pulse for the stepper controller
  else if (throttle_counter_left_motor == 2)PORTD &= 0b11111011;            //Set output 2 low because the pulse only has to last for 20us

  //right motor pulse calculations
  throttle_counter_right_motor = throttle_counter_right_motor + 1;                                         //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttle_counter_right_motor > throttle_right_motor_memory) {         //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if (throttle_right_motor_memory < 0) {                                  //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if (throttle_counter_right_motor == 1)PORTD |= 0b00010000;           //Set output 4 high to create a pulse for the stepper controller
  else if (throttle_counter_right_motor == 2)PORTD &= 0b11101111;           //Set output 4 low because the pulse only has to last for 20us
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

void drawbattvolt() {
  float avgbattvolt;
  for (int i = 0; i <= 20 ; i++) {
    double Vcc;
    Vcc = readVcc() / 1000.0;
    unsigned int  ADCforV = analogRead(A1);
    battvolt = ADCforV * Vcc / 1024;
    avgbattvolt += battvolt;
  }
  battvolt = avgbattvolt / 25;
  //Serial.println(battvolt);
    if (battvolt < 3.2) {
    pcf8574.digitalWrite(P0, HIGH);
    pcf8574.digitalWrite(P1, HIGH);
    pcf8574.digitalWrite(P2, LOW);
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
  }
  if (battvolt >= 3.2 && battvolt < 3.63) {
    pcf8574.digitalWrite(P0, LOW);
    pcf8574.digitalWrite(P1, HIGH);
    pcf8574.digitalWrite(P2, HIGH);
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
  }
  if (battvolt >= 3.63 && battvolt < 3.8) {
    pcf8574.digitalWrite(P0, LOW);
    pcf8574.digitalWrite(P1, LOW);
    pcf8574.digitalWrite(P2, HIGH);
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
  }
  if (battvolt >= 3.8 && battvolt < 3.95) {
    pcf8574.digitalWrite(P0, LOW);
    pcf8574.digitalWrite(P1, LOW);
    pcf8574.digitalWrite(P2, LOW);
    pcf8574.digitalWrite(P4, HIGH);
    pcf8574.digitalWrite(P5, HIGH);
  }
  if (battvolt >= 3.95 && battvolt < 4.12) {
    pcf8574.digitalWrite(P0, LOW);
    pcf8574.digitalWrite(P1, LOW);
    pcf8574.digitalWrite(P2, LOW);
    pcf8574.digitalWrite(P4, LOW);
    pcf8574.digitalWrite(P5, HIGH);
  }
  if (battvolt >= 4.12) {
    pcf8574.digitalWrite(P0, LOW);
    pcf8574.digitalWrite(P1, LOW);
    pcf8574.digitalWrite(P2, LOW);
    pcf8574.digitalWrite(P4, LOW);
    pcf8574.digitalWrite(P5, LOW);
  }
}
