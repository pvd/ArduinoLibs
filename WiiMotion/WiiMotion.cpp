#include "WiiMotion.h"
//#include "Arduino.h"
//#include "WProgram.h"

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
  m_gyro.sampleTime = 0;
  m_gyro.p = 0; 
  m_gyro.r = 0; 
  m_gyro.y = 0; 
  
  m_accel.ipitch = 0;
  m_accel.iroll  = 0;
  
  // maybe get the accel zero value by averaging of 600 sample or so
  m_accel.ox = 480;
  m_accel.oy = 516;
  m_accel.oz = 504;
    
  float zp = 0.0;
  float zr = 0.0;
  float zy = 0.0;
   
  for(int i=0; i < 600; i++)
  {
    if ( getSensorData() == 1 )
    {
			zp += m_gyro.ip / 300.0;
			zr += m_gyro.ir / 300.0;
			zy += m_gyro.iy / 300.0;
    }
    
    delayMicroseconds(WMINTERLEAVE);
  }
  
  m_gyro.zp = zp;
  m_gyro.zr = zr;
  m_gyro.zy = zy;
    
  // Wait for the accel data  
  if ( getSensorData() == 2 )
    getSensorData();
  
  // Calculate the accel roll and pitch and use this a initial value for the roll and pitch
  m_accel.CalcRollAndPitch();
    
  m_pitch = m_accel.ipitch;
  m_roll  = m_accel.iroll;
}

// Get the gyro or accel raw data from the sensors
// The WiiMotion plus returns the gyro & accel data interleaved
// return value 0 = no update
// return value 1 = gyro data is updated
// return value 2 = accel data is updated
int WiiMotion::getSensorData()
{
  unsigned long curTime = micros();
  unsigned long cycleTime = curTime - m_prevTime; // in us
  if ( cycleTime < DT )
  {
    // ignore this update call and wait for the next,
    // we need a minimum of DT between sample updates
    return 0;
  }
  
  static uint8_t buf[6];   // array to store nunchuck data,
  boolean okWM = 0;
  boolean okNU = 0;
  
  Wire.beginTransmission(0x52);// transmit to device 0x52<<1=0xA4
  Wire.send(0x00);// sends one byte
  Wire.endTransmission();// stop transmitting

  Wire.requestFrom(0x52, 6);// request data 
  
  for(int i=0; i < 6; i++)
  {
    buf[i] = Wire.receive();
  }
    
  // The gyro and accel data is interleaved
  // Bit 1 of byte 5 determines if the data returned is gyro or accel data
  if (buf[5] & 0x2)
  { 
    // motion plus data
    m_gyro.sampleTime = cycleTime / 1e6;

    m_gyro.ip = buf[2] | ((buf[5] >> 2) << 8); 
    m_gyro.ir = buf[1] | ((buf[4] >> 2) << 8); 
    m_gyro.iy = buf[0] | ((buf[3] >> 2) << 8);       
    
    m_gyro.slowp = (buf[3] & 0x1) >> 0;
    m_gyro.slowr = (buf[4] & 0x2) >> 1;
    m_gyro.slowy = (buf[3] & 0x2) >> 1;
    
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
    
  m_prevTime = curTime;

  return okWM ? 1 : (okNU * 2);
}
  
void WiiMotion::update()
{
  int sensorUpdated = getSensorData();
  if (sensorUpdated == 1)
  {    
    m_gyro.CalcRollAndPitch();    
  }
  else if (sensorUpdated == 2)
  {
    m_accel.CalcRollAndPitch();
  }

  m_roll = GYROPRI * (m_roll + m_gyro.r * m_gyro.sampleTime) + (1-GYROPRI) * m_accel.iroll; 
  m_pitch = GYROPRI * (m_pitch + m_gyro.p * m_gyro.sampleTime) + (1-GYROPRI) * m_accel.ipitch;
}

