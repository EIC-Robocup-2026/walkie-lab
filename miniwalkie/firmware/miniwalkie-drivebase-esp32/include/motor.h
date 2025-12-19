#ifndef MOTOR_I2C_H
#define MOTOR_I2C_H

#include <Wire.h>
#include <Arduino.h>

class MotorI2C {       // The class
  private:             // Access specifier
    int driverAddress;
    int motorAddress;
    bool swap_dir = false;
  public:
    MotorI2C( int driverAddress, int motorAddress)
    {
      this->driverAddress = driverAddress;
      this->motorAddress = motorAddress;
    }
    MotorI2C( int driverAddress, int motorAddress , bool swap_dir)
    {
      this->driverAddress = driverAddress;
      this->motorAddress = motorAddress;
      this->swap_dir = swap_dir;
    }

    void run( float pwm )
    {
      Wire.beginTransmission( this->driverAddress );
      Wire.write( this->motorAddress );
      int dir;
      if( swap_dir ) dir = pwm > 0  ? 1 : 2;
      else dir = pwm > 0  ? 2 : 1;

      if (pwm == 0) { dir = 3; }
      Wire.write(dir);
      Wire.write(int(abs(pwm)));
      Wire.endTransmission();
    }

    
    void Swap()
    {
      this->swap_dir = !this->swap_dir;
    }
    void Swap( bool dir )
    {
      this->swap_dir = dir;
    }
    int dir()
    {
      return this->swap_dir;
    }
};

#endif // MOTOR_I2C_H