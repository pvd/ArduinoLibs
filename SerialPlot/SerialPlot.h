#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class SerialPlot
{
private:
  String m_name;
  String m_color;
  int    m_sampleCnt;
  
public:
  SerialPlot(char * name, char * color, int sampleCnt)
  {
    m_name  = name;
    m_color = color;
    m_sampleCnt = sampleCnt;
  }
  
  void AddSample(double xValue, double yValue)
  {
    Serial.print("sample:");
    Serial.print(m_name);
    Serial.print(",");
    Serial.print(xValue);
    Serial.print(",");
    Serial.println(yValue);
  }
  
  void AddSample(double yValue)
  {
    AddSample(millis(), yValue);
  }
  
  void Begin()
  {
    Serial.print("curve:");
    Serial.print(m_name);
    Serial.print(",");
    Serial.print(m_color);
    Serial.print(",");
    Serial.println(m_sampleCnt);
  }
};
