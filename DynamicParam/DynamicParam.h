#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define MAX_NR_PARAMS      10
#define SERIAL_RX_BUF_SIZE 25

class DynamicParam
{
public:
  String   m_name;
  double   m_minValue;
  double   m_maxValue;
  double * m_valueRef;

  DynamicParam()
  {
    m_valueRef = NULL;
  }
  
  void Assign(const String & name, double minValue, double maxValue, double * valueRef) 
  {
    m_name     = name;
    m_minValue = minValue;
    m_maxValue = maxValue;
    m_valueRef = valueRef;
  }
  
  void Begin()
  {
    Serial.print("param:");
    Serial.print(m_name);
    Serial.print(",");
    Serial.print(m_minValue);
    Serial.print(",");
    Serial.print(m_maxValue);
    Serial.print(",");
    Serial.println(*m_valueRef);
  }
};


class DynamicParamManager
{
private:
  int          m_paramCnt;
  DynamicParam m_params[MAX_NR_PARAMS];
  char         m_serialRxBuf[SERIAL_RX_BUF_SIZE];
  
public:
  DynamicParamManager()
  {
    m_paramCnt = 0;
  }
  
  void Begin()
  {
    for ( int i = 0; i < m_paramCnt; i++ )
    {
      m_params[i].Begin();
    }
  }
  
  void AddParam(char * name, double minValue, double maxValue, double * valueRef)
  {
    if ( m_paramCnt >= MAX_NR_PARAMS )
      return;
      
    m_params[m_paramCnt++].Assign(name, minValue, maxValue, valueRef);
  }
  
  void Update()
  {
    int  bufIdx   = 0;
    int  paramIdx = -1;
    enum states {waiting4Cmd, waiting4Name, waiting4Value};
    enum states state = waiting4Cmd;
    
    digitalWrite(13, 0); // digital 13 used as parse error indicator
    while ( (m_serialRxBuf[bufIdx] = Serial.read()) != -1 )
    {
      if ( bufIdx >= SERIAL_RX_BUF_SIZE )
      { 
        // ERROR, command to big for receiption buffer
        digitalWrite(13, 1); 
    	  Serial.flush();
        break;
      }
    
      switch ( state )
      {
        case waiting4Cmd:
        {
    	  	if ( m_serialRxBuf[bufIdx] == ':' )
    	  	{
    	  	  m_serialRxBuf[bufIdx] = '\0';
    	  	  if ( strcmp((const char*)m_serialRxBuf, (const char*)"param") == 0 )
    	  	  {
    	  	    bufIdx = 0;
    	  	    state = waiting4Name;
    	  	  }
    	  	  else
    	  	  {
     	  	    // ERROR, unknown cmd
              digitalWrite(13, 1); 

    	  	    Serial.flush();
    	  	    break;
    	  	  }
    	  	}
    	  	else
    	  	{
    	  	  bufIdx++;
    	  	}
        } break;
        case waiting4Name:
        {
          if ( m_serialRxBuf[bufIdx] == ',' )
          {
            m_serialRxBuf[bufIdx] = '\0';
            // search the list of parameter for the name
            for ( int i = 0; i < m_paramCnt; i++ )
            {
              if ( m_params[i].m_name.compareTo(m_serialRxBuf) == 0 )
              {
                paramIdx = i;
              	break;
              }
            }
            
            if ( paramIdx != -1 )
            {
              state = waiting4Value;
              bufIdx = 0;
            }
            else
            {
              // ERROR, unknown parameter name
              digitalWrite(13, 1); 
              Serial.flush();
              break;
            }
          }
          else
          {
	          bufIdx++;
	        }
        } break;
        case waiting4Value:
        {
          if ( m_serialRxBuf[bufIdx] == '\n' )
          {
          	m_serialRxBuf[bufIdx] = '\0';
            *m_params[paramIdx].m_valueRef = atof(m_serialRxBuf);
            break;
          }
          else
          {
            bufIdx++;
          }
        } break;
      }
    }
  }
};

