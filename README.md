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