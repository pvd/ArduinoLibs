#include "WiiMotion.h"
#include "math_ex.h"
//#include "Arduino.h"
#include "WProgram.h"

WiiMotion::WiiMotion()
{

}

void WiiMotion::begin()
{
  Wire.begin();                // join i2c bus as master

  Wire.beginTransmission (0x53);	// transmit to device id 0x53<<1=0xA6
  Wire.send(0xFE);		// sends memory address
  Wire.send(0x05);		// *(0xA600FE)=0x05
  Wire.endTransmission ();	// Nunchuck pass-through mode
  delay(250);
  
  Wire.beginTransmission(0x53);	// transmit to device id 0x53<<1=0xA6 
  Wire.send(0xF0);		// sends memory address
  Wire.send(0x55);		// *(0xA600F0)=0x55
  Wire.endTransmission();	// activate motion plus
  delay(250);
  
  m_prevTime = micros();
  reset();
}
  
void WiiMotion::reset()
{
  m_gyro.p = 0; 
  m_gyro.r = 0; 
  m_gyro.y = 0; 
  
  m_accel.ipitch = 0;
  m_accel.iroll  = 0;
  
  float ox = 0.0;
  float oy = 0.0;
  float oz = 0.0;
  for(int i=0; i < 10; i++)
  {
    update();
    
    delayMicroseconds(WMINTERLEAVE);
  }
    
  m_accel.ox = 480;
  m_accel.oy = 516;
  m_accel.oz = 504;
    
  float zp = 0.0;
  float zr = 0.0;
  float zy = 0.0;
    
  for(int i=0; i < 600; i++)
  {
    update();
  
    zp += m_gyro.ip / 600.0;
    zr += m_gyro.ir / 600.0;
    zy += m_gyro.iy / 600.0;
  
    delayMicroseconds(WMINTERLEAVE);
  }
  
  m_gyro.zp = zp;
  m_gyro.zr = zr;
  m_gyro.zy = zy;
    
  m_pitch = m_accel.ipitch;
  m_roll  = m_accel.iroll;
}
  
void WiiMotion::update()
{
  unsigned long curTime = micros();
  unsigned long cycleTime = curTime - m_prevTime; // in us
  if ( cycleTime < DT )
  {
    // ignore this update call and wait for the next,
    // we need a minimum of DT between sample updates
    return;
  }
  
  m_dt = cycleTime / 1e6;

  static uint8_t buf[6];   // array to store nunchuck data,
  boolean okWM = 0;
  boolean okNU = 0;
  
  Wire.beginTransmission(0x52);// transmit to device 0x52<<1=0xA4
  Wire.send((uint8_t)0x00);// sends one byte
  Wire.endTransmission();// stop transmitting

  Wire.requestFrom(0x52, 6);// request data 
  
  for(int i=0; i < 6; i++)
  {
    buf[i] = Wire.receive();
  }
    
  if (buf[5] & 0x2)
  { 
    // motion plus data
    m_gyro.ip = buf[2] | ((buf[5] >> 2) << 8); 
    m_gyro.ir = buf[1] | ((buf[4] >> 2) << 8); 
    m_gyro.iy = buf[0] | ((buf[3] >> 2) << 8);       
  
    // FOLLOW the right hand rule
    // front is x axis
    // right is y axis
    // up is z axis
    //
    // using 2.72mV/deg/s and 1.35V reference 
    // 8192 is 594deg/s (1.35/0.00272)
    float p = GYROPDIR(m_gyro.ip - m_gyro.zp) * 594.0 / 8192.0;
    float r = GYRORDIR(m_gyro.ir - m_gyro.zr) * 594.0 / 8192.0;
    float y = GYROYDIR(m_gyro.iy - m_gyro.zy) * 594.0 / 8192.0;
  
    m_gyro.slowp = (buf[3] & 0x1) >> 0;
    m_gyro.slowr = (buf[4] & 0x2) >> 1;
    m_gyro.slowy = (buf[3] & 0x2) >> 1;
  
    if (! m_gyro.slowp) // pitch fast?
      p /= 0.22;
  
    if (! m_gyro.slowr) // roll fast?
      r /= 0.22;
  
    if (! m_gyro.slowy) // yaw fast?
      y /= 0.22;
  
    m_gyro.p = m_gyro.p * GYRALPHA + (1 - GYRALPHA) * p;
    m_gyro.r = m_gyro.r * GYRALPHA + (1 - GYRALPHA) * r;
    m_gyro.y = m_gyro.y * GYRALPHA + (1 - GYRALPHA) * y;
  
    okWM = 1;    
  }
  else //if (buf[0]&buf[1]&0x80) // read analog stick high bit MUST be 1 for SX,SY
  {
    // nunchuck data
    m_accel.ix = ((buf[2] << 2)          | ((buf[5] >> 3) & 0x2)); // 508 center
    m_accel.iy = ((buf[3] << 2)          | ((buf[5] >> 4) & 0x2)); // 494 196 = 1g
    m_accel.iz = (((buf[4] & 0xfe) << 2) | ((buf[5] >> 5) & 0x6)); // 514 300-726

    okNU = 1; 
  }  

  float x = ACCXDIR(m_accel.ix - m_accel.ox); 
  float y = ACCYDIR(m_accel.iy - m_accel.oy); 
  float z = ACCZDIR(m_accel.iz - m_accel.oz); 
  
  m_accel.ipitch = arctan2(y, sqrt(x * x + z * z) ) * 180 / PI;
  m_accel.iroll  = arctan2(x, sqrt(y * y + z * z) ) * 180 / PI;
  
  m_accel.x = x;
  m_accel.y = y;
  m_accel.z = z;
  
  m_roll = GYROPRI * (m_roll + m_gyro.r * m_dt) + (1-GYROPRI) * m_accel.iroll; 
  m_pitch = GYROPRI * (m_pitch + m_gyro.p * m_dt) + (1-GYROPRI) * m_accel.ipitch;
  
  m_prevTime = curTime;
}