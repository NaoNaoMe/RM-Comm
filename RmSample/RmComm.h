#ifndef RM_COMM_H
#define RM_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define RM_SUPPORT_64BIT

#ifdef __AVR__              //This statement for AVR
#define RM_ADDRESS_2BYTE
#elif defined(__arm__)      //This statement for ARM
#define RM_ADDRESS_4BYTE
#else
#define RM_ADDRESS_4BYTE    //You can change address width according to your application
#endif

/* Raw received data buffer size for rx-irq*/
#define RM_UART_BUFFER_SIZE     (1 << 4)    /* buffer size  2^n (n:2-7) */

#define  RM_DATA_NULL            (uint8_t *)0x00000000

/* Definitions of return status values */
#define RM_STATUS_ERR           0
#define RM_STATUS_SUCCESS       1

/* Definitions of return buffer state */
#define RM_BUFFER_OK            1
#define RM_BUFFER_FULL          0

#define RM_TRUE                 1
#define RM_FALSE                0


typedef struct
{
#ifdef RM_ADDRESS_4BYTE
    uint32_t Address;
    uint8_t  Length;
#else
    uint16_t Address;
    uint8_t  Length;
#endif
} RM_BYPASS_RESPONSE;


typedef RM_BYPASS_RESPONSE (*RM_FUNCPTR)(uint8_t payload[], uint8_t payloadsize);

/*-- begin: functions --*/
void        RM_Run( void );
uint8_t     RM_DecodeRcvdData( uint8_t rcvBuffer[], uint8_t* rcvdLength, uint8_t maxLength);
void        RM_TaskCore( uint8_t isRcvSuccess, uint8_t rcvBuffer[], uint8_t* rcvdLength );

void        RM_Initialize( uint8_t version[], uint8_t versionSize, uint8_t base_cycle, uint32_t passkey );
void        RM_AttachBypassFunction( RM_FUNCPTR func );

void        RM_AttachTransmitBuffer(uint8_t tmp[], uint8_t size);

uint16_t    RM_SetReceivedData( uint8_t abyte );
uint8_t*    RM_TryTransmission( void );
uint8_t*    RM_GetTransmitData( void );
uint8_t     RM_IsTransmissionBusy();
uint8_t     RM_Connected();

#ifdef __cplusplus
}
#endif

#endif  /* RM_COMM_H */

/*-- end of file --*/
