# BalanceBot
Code and resources for "BalanceBot", an Arduino-powered two-wheel balancing robot.

  * Project page: <http://rogerignazio.com/projects/balancebot>
  * Code: <https://github.com/rji/balancebot>

## Hardware
  * Arduino Uno
  * 2x Creative Robotics HUB-ee (120:1)
  * STMicroelectronics LPY503AL gyro
  * Analog Devices ADXL335 accelerometer

## Software
Code for this project was developed and tested using the Arduino IDE, version 1.0.4, and the Arduino Uno, Rev 3. 

The accuracy of the angle calculation is improved with a complementary filter.

Motor output is calculated using a PID controller. Loop tuning was accomplished using (mostly) the Ziegler-Nichols method, with minor corrections to the values based on observations. To tune, modify the constants `kp`, `ki`, and `kd`.

## Additional Reading
Colton, Shane. _[The Balance Filter: A Simple Solution for Integrating Accelerometer and Gyroscope Measurements for a Balancing Platform](http://web.mit.edu/scolton/www/filter.pdf)_. Accessed May 2013.

[PID Controller](http://en.wikipedia.org/wiki/PID_controller). _Wikipedia_. Accessed May 2013.
