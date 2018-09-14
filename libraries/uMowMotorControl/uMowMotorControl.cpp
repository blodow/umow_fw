#include "uMowMotorControl.h"

// Constructors ////////////////////////////////////////////////////////////////

uMowMotorControl::uMowMotorControl()
{
  //Pin map
  _INA1 = 12;
  _INB1 = 11;
  //_EN1DIAG1 = 6;
  //_CS1 = A0; 
  _INA2 = 9;
  _INB2 = 6;
  //_EN2DIAG2 = 12;
  //_CS2 = A1;
}

// Public Methods //////////////////////////////////////////////////////////////
void uMowMotorControl::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_INA1,OUTPUT);
  pinMode(_INB1,OUTPUT);
  //pinMode(_EN1DIAG1,INPUT);
  //pinMode(_CS1,INPUT);
  pinMode(_INA2,OUTPUT);
  pinMode(_INB2,OUTPUT);
  //pinMode(_EN2DIAG2,INPUT);
  //pinMode(_CS2,INPUT);
  pwmTimers.init();
  pwmTimers.setDutyCycle(0,0);
}

// Set speed for motor 1, speed is a number betwenn -400 and 400
void uMowMotorControl::setM1Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 512)  // Max PWM dutycycle
    speed = 512;
  pwmTimers.setDutyCycle1(speed);
  if (speed == 0)
  {
    digitalWrite(_INA1,LOW);   // Make the motor coast no
    digitalWrite(_INB1,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA1,LOW);
    digitalWrite(_INB1,HIGH);
  }
  else
  {
    digitalWrite(_INA1,HIGH);
    digitalWrite(_INB1,LOW);
  }
}

// Set speed for motor 2, speed is a number betwenn -400 and 400
void uMowMotorControl::setM2Speed(int speed)
{
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
  if (speed > 512)  // Max 
    speed = 512;
  pwmTimers.setDutyCycle2(speed);
  if (speed == 0)
  {
    digitalWrite(_INA2,LOW);   // Make the motor coast no
    digitalWrite(_INB2,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(_INA2,LOW);
    digitalWrite(_INB2,HIGH);
  }
  else
  {
    digitalWrite(_INA2,HIGH);
    digitalWrite(_INB2,LOW);
  }
}

// Set speed for motor 1 and 2
void uMowMotorControl::setSpeeds(int m1Speed, int m2Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}

// Brake motor 1, brake is a number between 0 and 400
void uMowMotorControl::setM1Brake(int brake)
{
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 512)  // Max brake
    brake = 512;
  digitalWrite(_INA1, LOW);
  digitalWrite(_INB1, LOW);
  pwmTimers.setDutyCycle1(brake);
}

// Brake motor 2, brake is a number between 0 and 400
void uMowMotorControl::setM2Brake(int brake)
{
  // normalize brake
  if (brake < 0)
  {
    brake = -brake;
  }
  if (brake > 512)  // Max brake
    brake = 512;
  digitalWrite(_INA2, LOW);
  digitalWrite(_INB2, LOW);
  pwmTimers.setDutyCycle1(brake);
}

// Brake motor 1 and 2, brake is a number between 0 and 400
void uMowMotorControl::setBrakes(int m1Brake, int m2Brake)
{
  setM1Brake(m1Brake);
  setM2Brake(m2Brake);
}

void uMowMotorControl::setMowSpeed(int speed) {
  pwmTimers.setDutyCycleMow(speed);
}

//// Return motor 1 current value in milliamps.
//unsigned int uMowMotorControl::getM1CurrentMilliamps()
//{
//  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
//  return analogRead(_CS1) * 34;
//}
//
//// Return motor 2 current value in milliamps.
//unsigned int uMowMotorControl::getM2CurrentMilliamps()
//{
//  // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
//  return analogRead(_CS2) * 34;
//}
//
//// Return error status for motor 1 
//unsigned char uMowMotorControl::getM1Fault()
//{
//  return !digitalRead(_EN1DIAG1);
//}
//
//// Return error status for motor 2 
//unsigned char uMowMotorControl::getM2Fault()
//{
//  return !digitalRead(_EN2DIAG2);
//}
