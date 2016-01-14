#include <MsTimer2.h>	//MsTimer2 Library

#include <RmComm.h>

char VersionInfo[] = "RmSample";

boolean rmTrg = false;

boolean cntFlg = false;
int count = 0;

void timerFire() {
  rmTrg = true;
  
}

void setup() {
  // initialize serial:
  Serial.begin(9600);

  RM_Initial((uint8_t*)VersionInfo, sizeof(VersionInfo));
  
  MsTimer2::set(5, timerFire);
  MsTimer2::start();
  
}

void loop() {
  
  if( rmTrg == true )
  {
    rmTrg = false;

    if(cntFlg== false)
    {
      count++;
    
    }

    serialRxEvent();
    
    RM_Communiation();
    
    serialTxEvent();
    
  }
}

void serialRxEvent() {
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    
    RM_PutReceiveBuff(inChar);
    
  }
}

void serialTxEvent() {
    uint8_t *psndChar = RM_GetSendSoFrame();
    
    int i = 0;
    uint8_t sndBuff[256];
    while( psndChar != RM_RET_NULL )
    {
      sndBuff[i] = *psndChar;
      i++;
      
      psndChar = RM_GetSendFrame();
      
      if( psndChar == RM_RET_NULL )
      {
        Serial.write(sndBuff, i);
        break;
        
      }
      
    }
}



