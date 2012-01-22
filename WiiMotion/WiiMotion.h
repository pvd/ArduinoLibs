// Code "stolen" from http://www.elenafrancesco.org/old/arduino/baronpilot/

#ifndef WII_MOTION_H
#define WII_MOTION_H

#include <WProgram.h>
#include <math.h>
#include <Wire.h>
#include <math_ex.h>

#define GYROPRI         0.998 
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

struct TGyro
{
  float sampleTime;
  float p,r,y;    // current 
  float zp,zr,zy; // zero 
  float ip,ir,iy; // istant 
  char  slowp,slowr,slowy;
  
  void CalcRollAndPitch()
  {
		// using 2.72mV/deg/s and 1.35V reference 
		// 8192 is 594deg/s (1.35/0.00272)
		float p = GYROPDIR(ip - zp) * 594.0 / 8192.0;
		float r = GYRORDIR(ir - zr) * 594.0 / 8192.0;
		float y = GYROYDIR(iy - zy) * 594.0 / 8192.0;
		
		if (! slowp) // pitch fast?
			p /= 0.22;
		
		if (! slowr) // roll fast?
			r /= 0.22;
		
		if (! slowy) // yaw fast?
			y /= 0.22;
		
		p = p * GYRALPHA + (1 - GYRALPHA) * p;
		r = r * GYRALPHA + (1 - GYRALPHA) * r;
		y = y * GYRALPHA + (1 - GYRALPHA) * y;  
  }
}; 

struct TAccel
{
  int   ix,iy,iz;
  float ox,oy,oz;
  float x,y,z;
  float ipitch,iroll;
  
  void CalcRollAndPitch()
  {
		x = ACCXDIR(ix - ox); 
		y = ACCYDIR(iy - oy); 
		z = ACCZDIR(iz - oz); 
		
		ipitch = arctan2(y, sqrt(x * x + z * z) ) * 180 / PI;
		iroll  = arctan2(x, sqrt(y * y + z * z) ) * 180 / PI;
  }
}; 

// WiiMotion
class WiiMotion
{
private:
  TGyro  m_gyro;
  TAccel m_accel;
  
  unsigned long m_prevTime;  
  float  m_pitch;
  float  m_roll;

  int getSensorData();
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
};

#endif