#include <SPI.h>
#include "RF24.h"
#include "Arduino.h"
#include <I2C.h>
#include <Battery.h>
#include "Timer.h"

#define DRIVERS_ENABLEPIN 6
#define SPEAKER_PIN 7
#define LASER_PIN 8
#define BATTERYSENSE_PIN A0

Battery battery(3000, 4200, BATTERYSENSE_PIN);

// (Create an instance of a radio, specifying the CE and CS pins. )
RF24 myRadio (9, 10); // "myRadio" is the identifier you will use in following methods
Timer t;

byte addresses[][6] = {"1Node", "2Node"};

int gyro_address = 0x68;                                    //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = -500;                           //Enter the accelerometer calibration value       (680)

//Various settings
float pid_p_gain = 1.81;                                    //Gain setting for the P-controller              (9.00)     (14.00)    (2.850)   (0.80)
float pid_i_gain = 0.05;                                    //Gain setting for the I-controller              (0.50)      (1.00)    (0.025)   (0.01)
float pid_d_gain = 0.05;                                    //Gain setting for the D-controller              (7.00)      (0.05)    (1.655)   (1.00)
float turning_speed = 42;                                   //Turning speed                                    (20)
float max_target_speed = 40;                                //Max target speed                                (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start;
int received_byte;
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value;
unsigned long loop_timer;
float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
int I2C_SEND_BYTE = 0;
int battlevel;
bool battlowflag = false;
bool battlow_sentToLEDsflag = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(DRIVERS_ENABLEPIN, OUTPUT);
  digitalWrite(DRIVERS_ENABLEPIN, HIGH);
  digitalWrite(SPEAKER_PIN, HIGH);
  delay(200);
  digitalWrite(SPEAKER_PIN, LOW);

  Serial.begin(115200);                                                   //Start the serial port at 115200 kbps
  I2c.begin();
  I2c.setSpeed(1);
  I2c.pullup(1);
  I2c.timeOut(7);
  I2C_SEND_BYTE = 0;
  I2c.write(8, I2C_SEND_BYTE);
  battery.begin(5000, 1.0, &asigmoidal);

  I2c.write(gyro_address, 0x6B, 0x00);
  I2c.write(gyro_address, 0x1B, 0x00);
  I2c.write(gyro_address, 0x1C, 0x08);
  I2c.write(gyro_address, 0x1A, 0x03);

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  for (receive_counter = 0; receive_counter < 500; receive_counter++) {     //Create 500 loops
    if (receive_counter % 15 == 0)digitalWrite(LASER_PIN, !digitalRead(LASER_PIN));         //Change the state of the LED every 15 loops to make the LED blink fast
    I2c.read(gyro_address, 0x43, 4);
    gyro_yaw_calibration_value += I2c.receive() << 8;
    gyro_yaw_calibration_value |= I2c.receive();
    gyro_pitch_calibration_value += I2c.receive() << 8;
    gyro_pitch_calibration_value |= I2c.receive();
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
  //myRadio.setPALevel(RF24_PA_MAX);                                        //Uncomment for more power
  //myRadio.setDataRate(RF24_2MBPS);
  myRadio.setDataRate(RF24_250KBPS);
  myRadio.openWritingPipe( addresses[1]); // 2Node
  myRadio.openReadingPipe(1, addresses[0]); // 1Node
  myRadio.startListening();
  t.every(3000, readbattlevel);
  t.every(2000, sendbattlow_LEDS);
  t.every(5000, sendbattlow_RF);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  t.update();
  if ( myRadio.available()) // Check for incoming data from transmitter
  {
    while (myRadio.available())  // While there is data ready
    {
      myRadio.read( &received_byte, sizeof(received_byte) );
      Serial.println(received_byte, BIN);
      digitalWrite(LASER_PIN, HIGH);
      receive_counter = 0;                                                  //Reset the receive_counter variable
    }
  }
  if (receive_counter <= 100)receive_counter ++;                            //The received byte will be valid for 100 program loops (100 milliseconds)
  else {
    received_byte = 0x00;                                                   //After 100 milliseconds the received byte is deleted
    receive_counter = 0;                                                    //Reset the receive_counter variable
    digitalWrite(LASER_PIN, LOW);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  I2c.read(gyro_address, 0x3F, 2);
  accelerometer_data_raw = I2c.receive() << 8;
  accelerometer_data_raw |= I2c.receive();
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer

  if (start == 0 && angle_acc > -10 && angle_acc < 10) {                    //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
    digitalWrite(DRIVERS_ENABLEPIN, LOW);
  }

  I2c.read(gyro_address, 0x43, 4);
  gyro_yaw_data_raw = I2c.receive() << 8;
  gyro_yaw_data_raw |= I2c.receive();
  gyro_pitch_data_raw = I2c.receive() << 8;
  gyro_pitch_data_raw |= I2c.receive();

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;            //(0.000031)     //Calculate the traveled during this loop angle and add this to the angle_gyro variable

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  angle_gyro -= gyro_yaw_data_raw * +0.0000003;                             //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.185 ;     //(0.125)

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if (pid_i_mem > 400)pid_i_mem = 400;                                      //Limit the I-controller to the maximum controller output
  else if (pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > 400)pid_output = 400;                                    //Limit the PI-controller to the maximum controller output
  else if (pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if (pid_output < 5 && pid_output > -5)pid_output = 0;                     //Create a dead-band to stop the motors when the robot is balanced

  if (angle_acc > 87 || angle_acc < -87 || start == 0) {                    //If the robot tips over or the start variable is zero
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
    digitalWrite(DRIVERS_ENABLEPIN, HIGH);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if (received_byte & B00000001) {                                          //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if (received_byte & B00000010) {                                          //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }
  if (received_byte & B00000100) {                                          //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint > -2.5)pid_setpoint -= 0.9;                           //Slowly change the setpoint angle so the robot starts leaning forewards
    if (pid_output >= max_target_speed * -1)pid_setpoint -= 0.09;           //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if (received_byte & B00001000) {                                          //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if (pid_setpoint < 2.5)pid_setpoint += 0.9;                            //Slowly change the setpoint angle so the robot starts leaning backwards
    if (pid_output <= max_target_speed)pid_setpoint += 0.09;                //Slowly change the setpoint angle so the robot starts leaning backwards
  }

  if (received_byte & B00100000) {
  }

  if (received_byte & B00010000) {
    I2C_SEND_BYTE = battlevel;
    I2c.write(8, I2C_SEND_BYTE);
    received_byte = B00000000;
  }

  if (!(received_byte & B00001100)) {                                       //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if (pid_setpoint > 0.5)pid_setpoint -= 0.10;                            //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (pid_setpoint < -0.5)pid_setpoint += 0.10;                      //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0) {                                                  //If the setpoint is zero degrees
    if (pid_output < 0)self_balance_pid_setpoint += 0.02;                    //Increase the self_balance_pid_setpoint if the robot is still moving forewards (0.12)
    if (pid_output > 0)self_balance_pid_setpoint -= 0.02;                    //Decrease the self_balance_pid_setpoint if the robot is still moving backwards (0.12)
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
  throttle_counter_left_motor = throttle_counter_left_motor + 1;            //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
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
  throttle_counter_right_motor = throttle_counter_right_motor + 1;          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
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

void readbattlevel() {
  battlevel = map(battery.level(), 5, 85, 1, 6);
  if (battlevel == 1) {
    battlowflag = true;
  }
  else {
    battlowflag = false;
  }
}

void sendbattlow_LEDS() {
  if (battlowflag && battlow_sentToLEDsflag == false) {
    I2C_SEND_BYTE = battlevel;
    I2c.write(8, I2C_SEND_BYTE);
    Serial.println("Battery is low. Consider charging.");
    battlow_sentToLEDsflag = true;
  }
}

void sendbattlow_RF() {
  if (battlowflag && received_byte == 0x00) {
    myRadio.stopListening();                                    // First, stop listening so we can talk.
    myRadio.write(&battlevel, sizeof(int));
    myRadio.write(&battlevel, sizeof(int));
    myRadio.startListening();                                    // Now, continue listening
  }
}
