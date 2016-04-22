#include <MsTimer2.h>  //MsTimer2 Library

#include <RmComm.h>

#define LED_PIN 3
#define SW_PIN 4

char VersionInfo[] = "RmSample";

boolean rmTrg = false;

boolean cntFlg = false;
int count = 0;
boolean ctrlFlg = true;
int debounceCount = 0;
boolean ledEmitFlg = false;

void timerFire() {
  rmTrg = true;
  
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);
  
  // initialize serial:
  Serial.begin(9600);

  RM_Initial((uint8_t*)VersionInfo, sizeof(VersionInfo));
  
  MsTimer2::set(5, timerFire);
  MsTimer2::start();
  
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  
  if( rmTrg == true )
  {
    rmTrg = false;

    if(cntFlg== false)
    {
      count++;
    
    }

    if( ctrlFlg == true )
    {
      int inPin = digitalRead(SW_PIN);
     
      if (inPin == LOW)
      {
        debounceCount++;
        if( debounceCount > 200 )
        {
          debounceCount = 200;
          ledEmitFlg = true;
        }
      }
      else
      {
        debounceCount = 0;
        ledEmitFlg = false;
      }
    }

    if( ledEmitFlg == true )
    {
      digitalWrite(LED_PIN, LOW);
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
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
    
    while( psndChar != RM_RET_NULL )
    {
      Serial.write(*psndChar);
      psndChar = RM_GetSendFrame();
      
    }
}
