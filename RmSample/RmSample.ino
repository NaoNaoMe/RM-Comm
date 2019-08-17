#define RM_MONITOR

#ifdef RM_MONITOR
#include "RMonitor.h"
#else /* RM_MONITOR */
#include "RmComm.h"
#endif /* RM_MONITOR */

#define LED_PIN 3
#define SW_PIN 4

#define INTERVAL_CNT 5
#define RM_PASSWORD  0x0000FFFFU

#ifdef RM_MONITOR
RMonitor Rm;
#endif /* RM_MONITOR */


char VersionInfo[] = "RmSample";

static volatile boolean isCounting  = true;
static volatile int count = 0;

static volatile boolean isCtrl = true;
static volatile int debounceCount = 0;
static volatile boolean isEmitting = false;

unsigned long previousMillis = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT);
  
  // initialize serial:
  Serial.begin(9600);
  
#ifdef RM_MONITOR
  Rm.initialize((uint8_t*)VersionInfo, sizeof(VersionInfo), INTERVAL_CNT, RM_PASSWORD);
  Rm.print(F("RmSample"));
#else  /* RM_MONITOR */
  RM_Initialize((uint8_t*)VersionInfo, sizeof(VersionInfo), INTERVAL_CNT, RM_PASSWORD);
#endif  /* RM_MONITOR */

  digitalWrite(LED_PIN, HIGH);

}

void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= INTERVAL_CNT) {
    previousMillis = currentMillis;
    
    if(isCounting == true)
      count++;

    if( isCtrl == true )
    {
      int inPin = digitalRead(SW_PIN);
     
      if (inPin == LOW)
      {
        debounceCount++;
        if( debounceCount > 200 )
        {
          debounceCount = 200;
          isEmitting = true;
        }
      }
      else
      {
        debounceCount = 0;
        isEmitting = false;
      }
    }

    if( isEmitting == true )
      digitalWrite(LED_PIN, LOW);
    else
      digitalWrite(LED_PIN, HIGH);

#ifdef RM_MONITOR
    Rm.task();

    String receivedText = rmReceive();
    if(receivedText != "")
    {
        Rm.println(receivedText);
    }

#else  /* RM_MONITOR */
    serialRxEvent();
    
    RM_Run();
    
    serialTxEvent();
#endif  /* RM_MONITOR */


  }

}

#ifdef RM_MONITOR
String rmReceive()
{
    static String tmpText = "";
    String receivedText = "";
    char rc;

    while (Rm.available() > 0)
    {
        rc = Rm.read();

        if ((rc == 0x00) || (!isAscii(rc)))
        {
            continue;
        }

        if (rc == '\n')
        {
            receivedText = tmpText;
            tmpText = "";
            break;
        }
        else
        {
            tmpText += rc;
        }

    }

    return receivedText;
}

#else
void serialRxEvent() {
  while (Serial.available())
    RM_SetReceivedData((uint8_t)Serial.read());
}

void serialTxEvent() {
    uint8_t *pData = RM_TryTransmission();
    
    while( pData != RM_DATA_NULL )
    {
      Serial.write(*pData);
      pData = RM_GetTransmitData();
    }
}
#endif  /* RM_MONITOR */
