#include <RmComm.h>

#define LED_PIN 3
#define SW_PIN 4

#define INTERVAL_CNT 5
#define RM_PASSWORD  0x0000FFFFU

char VersionInfo[] = "RmSample";

volatile boolean cntFlg = false;
int count = 0;

volatile boolean ctrlFlg = true;

int debounceCount = 0;
boolean ledEmitFlg = false;

unsigned long previousMillis = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);
  
  // initialize serial:
  Serial.begin(9600);

  RM_Initial((uint8_t*)VersionInfo, sizeof(VersionInfo), INTERVAL_CNT, RM_PASSWORD);
  
  digitalWrite(LED_PIN, HIGH);

}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL_CNT) {
    previousMillis = currentMillis;
    
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