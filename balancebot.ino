// Filename:      balancebot.ino
// Description:   An Arduino-based two-wheel balancing robot.
// Author:        Roger Ignazio <me at rogerignazio dot com>
// Project page:  http://rogerignazio.com/projects/balancebot

// vim: set filetype=cpp tabstop=2 shiftwidth=2 softtabstop=2 expandtab:

// Include additional libraries
#include <math.h>

// Arduino UNO uses a 10-bit ADC yielding a max of 1024 values (0-1023)
#define ADC_RANGE 1023

// Math defines, for when more than 'float' precision is needed.
// On the Arduino Uno, both 'float' and 'double' are limited to 4 bytes.
#define PI 3.14159265359
#define RAD_TO_DEG 57.29577951308

// Operational defines
// Un-comment DEBUG to print additional information to the serial monitor
// Un-comment NOOP to disable PWM output to the motors
#define DEBUG
//#define NOOP

// Pin assignment
const int pin_pwm_motor_a = 5;        // PWM for motor a
const int pin_pwm_motor_b = 6;        // PWM for motor b
const int pin_dir_motor_a_in1 = 2;    // HUB-ee direction 1 (motor a)
const int pin_dir_motor_a_in2 = 4;    // HUB-ee direction 2 (motor a)
const int pin_dir_motor_b_in1 = 7;    // HUB-ee direction 1 (motor b)
const int pin_dir_motor_b_in2 = 8;    // HUB-ee direction 2 (motor b)
const int pin_gyro_z = A0;            // gyro reading
const int pin_acc_x = A1;             // accelerometer reading x-axis
const int pin_acc_z = A2;             // accelerometer reading z-axis

// Gyro, accelerometer, and time constants
const float gyro_volt = 3.3;          // LPY503AL operating voltage
const float gyro_idle = 1.23;         // LPY503AL zero-rate voltage
const float gyro_sens = 0.0083;       // LPY503AL sensitivity: 8.3mV/deg/sec
const float acc_volt = 3.3;           // ADXL335 operating voltage
const float acc_sens = 0.33;          // ADXL335 sensitivity: 330 mV/g
const float dt = 0.01;                // delta time; loop time in seconds

// PID tuning ... mostly followed the Ziegler-Nichols method for Classic PID.
// Note: this isn't perfect, and I don't have LabView or MATLAB to
// be able to improve it. I think it's a pretty good starting point though.
//
// Let ultimate gain 'Ku = 10', oscillation period 'Tu = 0.2'
// Kp = 0.6 * Ku;  Ki = 2 * Kp / Tu;  Kd = Kp * Tu / 8
const float kp = 6;                   // proportional gain
const float ki = 600;                 // integral gain
const float kd = 0.015;               // derivative gain

// The bot does not have a perfect 50/50 weight distribution. tilt_offset
// compensates for this by modifying the initial error value by a few degrees.
// Positive values cause the bot to tip forward; negative does the opposite.
const float tilt_offset = 3.6;

// Initialize other global variables
int gyro_z_offset, acc_x_offset, acc_z_offset;


void setup() {
  // I know it's not necessary to call pinMode() if using analogWrite(), or to
  // redefine INPUT pins, but have done so in the interest of completeness.
  pinMode(pin_pwm_motor_a, OUTPUT);
  pinMode(pin_pwm_motor_b, OUTPUT);
  pinMode(pin_dir_motor_a_in1, OUTPUT);
  pinMode(pin_dir_motor_a_in2, OUTPUT);
  pinMode(pin_dir_motor_b_in1, OUTPUT);
  pinMode(pin_dir_motor_b_in2, OUTPUT);
  pinMode(pin_gyro_z, INPUT);
  pinMode(pin_acc_x, INPUT);
  pinMode(pin_acc_z, INPUT);

  // Allow accelerometer and gyro to power-up and average the readings at idle
  // to determine a realistic offset. Ten samples should be enough.
  delay(100);
  int calibration_counter = 10;
  for (int i = 0; i < calibration_counter; i++) {
    gyro_z_offset += analogRead(pin_gyro_z);
    acc_x_offset += analogRead(pin_acc_x);
    acc_z_offset += analogRead(pin_acc_z);
    delay(10);
  }
  gyro_z_offset /= calibration_counter;
  acc_x_offset /= calibration_counter;
  acc_z_offset /= calibration_counter;
  acc_z_offset -= acc_sens / acc_volt * ADC_RANGE;

  analogWrite(pin_pwm_motor_a, 0);
  analogWrite(pin_pwm_motor_b, 0);

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println();
  Serial.println(" *** DEBUG MODE ON *** ");
  Serial.print("OFFSETS:\t");
  Serial.print(" gyro_z_offset: "); Serial.print(gyro_z_offset);
  Serial.print(" acc_x_offset: "); Serial.print(acc_x_offset);
  Serial.print(" acc_z_offset: "); Serial.print(acc_z_offset);
  Serial.println();
  Serial.print("PID GAINS:\t");
  Serial.print(" kp: "); Serial.print(kp);
  Serial.print(" ki: "); Serial.print(ki);
  Serial.print(" kd: "); Serial.print(kd);
  Serial.println();
#endif
}


void loop() {
  float gyro_rate, acc_x, acc_z, angle;
  float error, i, d, error_last;
  int speed;
  
  gyro_rate = (analogRead(pin_gyro_z) - gyro_z_offset) / (gyro_sens / gyro_volt * ADC_RANGE);
  acc_x = analogRead(pin_acc_x) - acc_x_offset;
  acc_z = analogRead(pin_acc_z) - acc_z_offset;

  // Convert to degrees and run values through a complementary filter.
  // atan2() will provide output in the range (-pi to pi); we add PI here
  // to make the "zero angle" setpoint 180 degrees.
  angle = (atan2(acc_x, acc_z) + PI) * RAD_TO_DEG;
  angle = 0.98 * (angle + gyro_rate * dt) + 0.02 * acc_x;

  // Output is handled by the PID controller below. Ideally,
  // error = (180 - angle). However, the weight distribution on the
  // bot is not balanced. We compensate for that with tilt_offset.
  error = (180 - tilt_offset) - angle;
  i += error * dt;
  d = (error - error_last) / dt;
  speed = (kp * error) + (ki * i) + (kd * d);
  error_last = error;

  // Positive 'speed' values are forward, negative are reverse. If
  // negative, motor direction is reversed and absolute value is used.
  //
  // For info on controlling the direction of the HUB-ee wheels, see
  // http://www.creative-robotics.com/bmdsresources
  if(speed >= 0) {
    digitalWrite(pin_dir_motor_a_in1, HIGH);
    digitalWrite(pin_dir_motor_a_in2, LOW);
    digitalWrite(pin_dir_motor_b_in1, LOW);
    digitalWrite(pin_dir_motor_b_in2, HIGH);
  }
  else {
    digitalWrite(pin_dir_motor_a_in1, LOW);
    digitalWrite(pin_dir_motor_a_in2, HIGH);
    digitalWrite(pin_dir_motor_b_in1, HIGH);
    digitalWrite(pin_dir_motor_b_in2, LOW);
    speed = abs(speed);
  }

#ifndef NOOP
  // Drive the motors if we're not in NOOP mode
  analogWrite(pin_pwm_motor_a, speed);
  analogWrite(pin_pwm_motor_b, speed);
#endif

#ifdef DEBUG
  Serial.print("gyro_rate: "); Serial.print(gyro_rate);
  Serial.print(" acc_x: "); Serial.print(acc_x);
  Serial.print(" acc_z: "); Serial.print(acc_z);
  Serial.print(" angle: "); Serial.print(angle);
  Serial.print(" error: "); Serial.print(error);
  Serial.print(" i: "); Serial.print(i);
  Serial.print(" d: "); Serial.print(d);
  Serial.print(" speed: "); Serial.print(speed);
  Serial.println();
#endif
}
