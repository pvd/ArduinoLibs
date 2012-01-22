// Code "stolen" from http://www.elenafrancesco.org/old/arduino/baronpilot/

#ifndef WII_MOTION_H
#define WII_MOTION_H

#include <math.h>
#include <Wire.h>

//#define GYROPRI         0.998 
#define GYROPRI         0.94 // gyroscope priority
#define GYRALPHA        0.8
#define ACCALPHA        0.6
#define WMINTERLEAVE    3000 // time that you must wait to read first wm+ then nunchuck
#define DT              3000 // Sample delta time 333Hz cycle time

// Direction of the sensor gyro
// + doesn't change direction
// - invert direction
// Pitch must have positive value when forward is up (negative otherwise)
// Roll must have positive value when left is up (negative otherwise)
// Yaw must have positive value when you rotate CCW anti-clock-wise 
#define GYROPDIR +
#define GYRORDIR +
#define GYROYDIR +

// Direction -/+ of the sensor accelerometer
// check this value with 'ai' command
// + doesn't change direction
// - invert direction
// X must have positive value when forward arm is up
// Y must have positive value when left arm is up
// Z must have positive value when vessel is not reversed up-down
#define ACCXDIR +
#define ACCYDIR +
#define ACCZDIR +

typedef struct
{
  float p,r,y;    // current 
  float zp,zr,zy; // zero 
  float ip,ir,iy; // istant 
  char  slowp,slowr,slowy;
} 
TGyro;

typedef struct
{
  int   ix,iy,iz;
  float ox,oy,oz;
  float x,y,z;
  float ipitch,iroll;
} 
TAccel;

// WiiMotion
class WiiMotion
{
private:
  TGyro  m_gyro;
  TAccel m_accel;
  
  unsigned long m_prevTime;  
  float  m_dt;
  float  m_pitch;
  float  m_roll;

public:
  WiiMotion();
  
  void begin();  
  void reset();
  void update();
  
  float roll()
  {
    return m_roll;
  }
  
  float pitch()
  {
    return m_pitch;
  }
  
  TAccel accelData()
  {
    return m_accel;
  }
  
  TGyro gyroData()
  {
    return m_gyro;
  }
  
  float deltatime()
  {
    return m_dt;
  }
};

#endif