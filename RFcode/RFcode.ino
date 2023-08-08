/*

  Radio Frequency Controller Code for Arduino (Remastered)
          By Sajeed Ahmed Galib Arnob
      Author of Original Code: Sheikh Saif Simran

  This code implements 2 BTS 79640 45A motor drivers to drive a remote
  controlled four-wheeled car.
  The car drives forward, backward, and steers left and right using all 4
  wheels.
  
  * The wireless receiver used is known as: Flysky FS-iA6B.
  * The wireless transmitter or remote can be either FS-GT2E, which is a
    gun-shaped controller, using a trigger and a steering wheel; or other
    Flysky remotes.

  In my current experiment, I have only tested and confirmed desired
  performance with the Flysky FS-GT2E and FS-TH9X controllers.

  For the FS-GT2E (gun-shaped controller), the trigger is used to drive
  forward and backward, and the steering wheel is used to turn left and right
  The combination of both is used to achieve driving and turning effects at the
  same time.
  The trigger and steering wheel of the controller are pressure sensitive,
  meaning that the slightest push/pull or turn can send a different signal
  in response to which, the car will move as instructed by this code.
  Therefore, speed control while driving is possible via only the amount
  of pressure or turning force applied to the trigger or the steering wheel
  respectively.

  The ST.REV and TH.REV switches can be used to reverse the polarity of the
  steering wheel and trigger respectively, meaning that left and right
  can be inverted for the steering wheel, and forward and backward can be
  inverted for the trigger.

  The ST.TRIM and TH.TRIM dials can be used to tune the neutral signal sent by
  the controller. Upon receiving this neutral signal, the car stops moving.
  It is important that the neutral signal is tuned by the dials so that
  the bot does not keep moving or turning without even touching the remote!

  The ST.DR dial is used to tune the sensitivity of the steering wheel so
  the turning speed of the car can be adjusted.

  This code is primarily written to be able to support the FS-GT2E controller.
  However, upon proper configuration of the remote, the FS-TH9X can also be used
  to control the car using the right joystick of the remote.
  The digital PWM pins used match with the pinout of Arduino Nano.
  However, by changing the pin numbers, other Arduino boards can also be used.

  To make this possible, the remote's channels and TRIM need to be configured
  properly.

  The gun-shaped controller's trigger uses channel 2 (in code, it's channel 1)
  and the steering wheel uses channel 1 (in code, it's channel 0)
  Similarly, the FS-TH9X remote's right joystick can be configured to send
  signals via different channels.
  
  For this code to work with the FS-TH9X remote, the right joystick must send
  forward and backward movement signals through channel 2 (on the remote's
  settings) and left and right turning signals through channel 1 (on the remote's
  settings)

  Finally, the receiver is capable of sending digital data to the Arduino via
  serial communication using the Arduino's RX pin.
  The three pins in a row (horizontally) on the right hand side of the receiver
  are as follows: Ground, VCC, Signal.
  The Signal pin must be connected to the Arduino's RX pin to enable
  Serial communication.
  The receiver uses IBus protocol to send signals to the Arduino
  via Serial communication through the Arduino's RX pin.
  Therefore, the IBusBM library must be installed to allow Arduino IDE
  to compile and upload this code to the Arduino.
*/

#include <IBusBM.h>


// Arduino pin numbers to be connected to the named pins of the motor driver
// used to control the wheels on the LEFT side of the remote controlled car
#define LEFT_ENL   7
#define LEFT_ENR   8
#define LEFT_LPWM  6  // pin 10 is a digital PWM pin on the Arduino Nano
#define LEFT_RPWM  5

// Arduino pin numbers to be connected to the named pins of the motor driver
// used to control the wheels on the RIGHT side of the remote controlled car
#define RIGHT_ENL  2
#define RIGHT_ENR  3
#define RIGHT_LPWM 11
#define RIGHT_RPWM 10


// declare a class handle for the IBusBM class imported from the IBusBM library
// the variable ibus will be used to read signals from the receiver's different
// channels
IBusBM ibus;

    // used to receive signals sent by the gun-shaped controller's trigger
int triggerChannel,

    // used to receive signals sent by the controller's steering wheel
    steerChannel;


// the motor driver's speed can be controlled using values from 0 to 255
// negative values down to -255 will be used to indicate the speed of the
// motors in backward direction.
int left_motors_speed, right_motors_speed;


void setup() {
  pinMode(LEFT_ENL, OUTPUT);
  pinMode(LEFT_ENR, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);

  pinMode(RIGHT_ENL, OUTPUT);
  pinMode(RIGHT_ENR, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);

  // The baud rate of the receiver is 115200, therefore the Arduino must also
  // receive the data sent to it by the RF receiver, at the same baud rate
  Serial.begin(115200);
  
  // the ibus instance begins to monitor the Serial communication of the
  // Arduino to be able to read and parse signals
  ibus.begin(Serial);
}


/*
 User-defined function to read from one of the channels of the receiver and then
 map the values to acceptable range of values.
 The receiver returns a value within the range 1000 and 2000.
 In my experiment, the best neutral position is 1500. Therefore, if the receiver
 gives a value 1500 to the Arduino, the motor's speed will be zero.
 If the value is less than 1500, then a value between -255 and 0 will be
 returned. This value will be used to indicate that the motor must rotate in
 the opposite direction. 

 @param channelInput: byte - Which channel number to read from
 @param minLimit: int - the minimum acceptable value for the motor's speed
 @param maxLimit: int - the maximum acceptable value for the motor's speed
 @param defaultValue: int - the value (usually 0) when the controller is
                            left untouched by the user

 @returns int - a value between the given range (minLimit and maxLimit) for the
                motor's speed
*/
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{  
  uint16_t channelIn = ibus.readChannel(channelInput);

  // After turning on the entire circuit for the remote-controlled car, the values
  // sent by the RF receiver, to the arduino are all less than 999. Therefore,
  // return defaultValue (in this case, 0) so that the car doesn't start moving
  // on it's own.
  if (channelIn < 999)
  {
    return defaultValue;    
  }

  // convert value sent by the RF receiver to a value between maxLimit and minLimit
  return map(channelIn, 1000, 2000, maxLimit, minLimit);
}


/*
 User-defined function to rotate left and right motors according to given speed

 @param left_speed: int - the speed of the motors on the left side of the car
                          if this value is negative, the motor will rotate in
                          opposite direction
 @param right_speed: int - the speed of the motors on the right side of the car
*/
void control_motors(int left_speed, int right_speed)
{
    // enable all channels of both the motor drivers
    digitalWrite(LEFT_ENL, HIGH);
    digitalWrite(LEFT_ENR, HIGH);
    digitalWrite(RIGHT_ENL, HIGH);
    digitalWrite(RIGHT_ENR, HIGH);

    // if left_speed is positive or zero, move motors in forward direction
    // motors will not move if speed is zero
    if (left_speed >= 0)
    {
        analogWrite(LEFT_LPWM, left_speed);
        analogWrite(LEFT_RPWM, 0);
    }
    // otherwise, rotate motors in opposite direction
    else
    {
        analogWrite(LEFT_LPWM, 0);
        analogWrite(LEFT_RPWM, abs(left_speed));
    }

    // same as left_speed but for right-side motors
    if (right_speed >= 0)
    {
        analogWrite(RIGHT_LPWM, right_speed);
        analogWrite(RIGHT_RPWM, 0);
    }
    else
    {
        analogWrite(RIGHT_LPWM, 0);
        analogWrite(RIGHT_RPWM, abs(right_speed));
    }
}


void loop() {

    steerChannel = readChannel(0, -255, 255, 0); // channel 1
    triggerChannel = readChannel(1, -255, 255, 0);  // channel 2
    Serial.print("Trigger Channel: ");
    Serial.println(triggerChannel);
    Serial.print("Steer Channel: ");
    Serial.println(steerChannel);

    // set initial speed of the motors before calculating the new speed set by
    // the remote-controller
    left_motors_speed = 0;
    right_motors_speed = 0;

    // speed for both motors for forward/backward movement
    left_motors_speed += triggerChannel;
    right_motors_speed += triggerChannel;

    // add/subtract steerChannel value from the motors' speeds to cause a
    // difference between left motor and right motor speeds to allow
    // forward/backward driving AND left/right turning at the same time
    left_motors_speed += steerChannel;
    right_motors_speed -= steerChannel;

    // the final value for the motors' speeds may exceed the acceptable range
    // so limit the values to their maximum and minimum limits
    left_motors_speed = constrain(left_motors_speed, -255, 255);
    right_motors_speed = constrain(right_motors_speed, -255, 255);


    Serial.print("Left motors speed: ");
    Serial.println(left_motors_speed);
    Serial.print("Right motors speed: ");
    Serial.println(right_motors_speed);

    // Use the user-defined functions to control the speeds of the motor to
    // enable driving of the car
    control_motors(left_motors_speed, right_motors_speed);

    // slight delay before processing the next signal
    delay(50);
}
