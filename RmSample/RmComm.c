//******************************************************************************
// RmComm(Generic)
// Version 3.0.0
// Copyright 2019 NaoNaoMe
//******************************************************************************

#include "RmComm.h"

/* all data are stored as Little-endian  */

/*-- begin: definitions --*/
#define RM_DEFAULT_BASE_CYCLE   5

#define RM_UART_BUFF_MASK       (RM_UART_BUFFER_SIZE - 1)

#define RM_LOG_FACTOR_MAX       32
#define RM_SND_PAYLOAD_SIZE     (RM_LOG_FACTOR_MAX*4)
#define RM_SND_FRAME_BUFF_SIZE  (RM_SND_PAYLOAD_SIZE+2)

#define RM_RCV_FRAME_BUFF_SIZE  32

#define RM_RCV_TIMEOUT_CNT      100
#define RM_REQ_TIMEOUT_CNT      2000
#define RM_SND_DEFAULT_CNT      500

#define RM_FUNC_NULL            (RM_FUNCPTR)0x00000000


/* Definitions of Serial Line Internet Protocol */
#define RM_FRAME_CHAR_END       0xC0
#define RM_FRAME_CHAR_ESC       0xDB
#define RM_FRAME_CHAR_ESC_END   0xDC
#define RM_FRAME_CHAR_ESC_ESC   0xDD

/* Definitions of communication mode */
#define RM_INITIAL_MODE         0
#define RM_LOG_MODE             1
#define RM_CANDR_MODE           2

/* Definitions of fixed frame index */
#define RM_FRAME_SEQCODE        0
#define RM_FRAME_PAYLOAD        1

/* Definitions of receiving state */
#define RM_SLIP_RCV_NOTHING     0
#define RM_SLIP_RCV_INCOMING    1
#define RM_SLIP_RCV_COMPLETE    2
#define RM_SLIP_RCV_FAULT       3

/* Definitions of SetLogDataFrame */
#define SETLOG_BIT_MASK         0xF0
#define SETLOG_START_BIT        0x10
#define SETLOG_END_BIT          0x20

/*-- begin: variables and constants --*/
#ifdef RM_ADDRESS_4BYTE
static uint32_t RM_BlockAddress;
static uint8_t RM_BlockLength;
static uint32_t RM_VersionAddress;
static uint8_t RM_VersionLength;
static uint8_t RM_LogSizeArray[RM_LOG_FACTOR_MAX];
static uint32_t RM_LogAddressArray[RM_LOG_FACTOR_MAX];
#else
static uint16_t RM_BlockAddress;
static uint8_t RM_BlockLength;
static uint16_t RM_VersionAddress;
static uint8_t RM_VersionLength;
static uint8_t RM_LogSizeArray[RM_LOG_FACTOR_MAX];
static uint16_t RM_LogAddressArray[RM_LOG_FACTOR_MAX];
#endif
static uint8_t RM_LogAvailableIndex;

static uint8_t RM_CommMode;
static uint8_t RM_IsApproved;
static uint8_t RM_IsTransmitPending;


static uint8_t RM_MasCnt;
static uint8_t RM_SlvCnt;

static uint8_t RM_RcvBufferArray[RM_RCV_FRAME_BUFF_SIZE];
static uint8_t RM_RcvBufferSize;

static uint8_t RM_SndBufferArray[RM_SND_FRAME_BUFF_SIZE];
static uint8_t* RM_SndBufferPtr;
static uint8_t RM_SndBufferSize;
static uint8_t RM_SndBufferSentIndex;

static uint16_t RM_RcvTimeoutCnt;
static uint16_t RM_ReqTimeoutCnt;
static uint16_t RM_TimingCnt;
static uint16_t RM_WaitCnt;

static uint8_t RM_RcvRingBuffArray[RM_UART_BUFFER_SIZE];
static uint8_t RM_RcvRingBuffEndIndex;
static uint8_t RM_RcvRingBuffBgnIndex;
static uint8_t RM_IsRcvSLIPActive;

static uint8_t RM_IsSndSLIPActive;
static uint8_t RM_IsSndBusy;

static uint8_t RM_BaseCycle;
static uint32_t RM_PassKey;


static RM_FUNCPTR RM_BypassFunctionPtr;



/*-- begin: prototype of function --*/

static void ManageSendData( uint8_t mode, uint8_t hasResponse, uint8_t masterCount );
static uint8_t AnalyzeReceivedFrame(uint8_t opcode, uint8_t payload[], uint8_t payloadSize, uint8_t* mode);
static uint16_t GetReceivedFrame( uint8_t payload[], uint8_t *rcvdLength, uint8_t maxLength );
static uint8_t  CtrlTransmitTiming( uint8_t mode, uint8_t isAssert );
static uint16_t GetLogData( uint8_t payload[], uint8_t *payloadSize );
static uint16_t GetBlockData( uint8_t payload[], uint8_t *payloadSize );
static uint8_t  GetCRC( uint8_t buffer[], uint8_t bufferSize );

static uint16_t SetLogStart( uint8_t payload[], uint8_t payloadSize );
static uint16_t SetLogStop( uint8_t payload[], uint8_t payloadSize );
static uint16_t SetTiming( uint8_t payload[], uint8_t payloadSize );
static uint16_t WriteData( uint8_t payload[], uint8_t payloadSize );
static uint16_t SetLogData( uint8_t payload[], uint8_t payloadSize );
static uint16_t ValidatePassKey( uint8_t payload[], uint8_t payloadSize );
static uint16_t SetDumpData( uint8_t payload[], uint8_t payloadSize );
static uint16_t SetBypassFunction( uint8_t payload[], uint8_t payloadSize );

/*-- begin: functions --*/

void RM_Run( void )
{
    uint8_t isSuccess;
    isSuccess = RM_DecodeRcvdData( RM_RcvBufferArray, &RM_RcvBufferSize, RM_RCV_FRAME_BUFF_SIZE );

    RM_TaskCore( isSuccess, RM_RcvBufferArray, &RM_RcvBufferSize );
}


void RM_TaskCore( uint8_t isRcvSuccess, uint8_t rcvBuffer[], uint8_t* rcvdLength )
{
    uint8_t hasResponse;

    uint8_t opcode;
    uint8_t  masterCount;
    uint8_t payloadSize;

    if(isRcvSuccess == RM_TRUE)
    {
        opcode = rcvBuffer[RM_FRAME_SEQCODE] & (uint8_t)0x0F;
        masterCount = rcvBuffer[RM_FRAME_SEQCODE] & (uint8_t)0xF0;
        payloadSize = *rcvdLength-1;   // delete opcode size

        hasResponse = AnalyzeReceivedFrame( opcode, &rcvBuffer[RM_FRAME_PAYLOAD], payloadSize, &RM_CommMode);

        if(hasResponse == RM_TRUE)
            RM_MasCnt = masterCount;

        *rcvdLength = 0;

    }
    else
    {
        hasResponse = RM_FALSE;
    }


    ManageSendData(RM_CommMode, hasResponse, RM_MasCnt);

}


uint8_t RM_DecodeRcvdData( uint8_t rcvBuffer[], uint8_t* rcvdLength, uint8_t maxLength)
{
    uint16_t result;

    result = GetReceivedFrame( rcvBuffer, rcvdLength, maxLength);

    if( result == RM_SLIP_RCV_FAULT ||
        result == RM_SLIP_RCV_NOTHING )
    {
        return RM_FALSE;
    }

    if( result == RM_SLIP_RCV_INCOMING )
    {
        RM_RcvTimeoutCnt += RM_BaseCycle;
        if( RM_RcvTimeoutCnt >= RM_RCV_TIMEOUT_CNT )
        {
            RM_RcvTimeoutCnt = 0;
            *rcvdLength = 0;
            RM_IsRcvSLIPActive = RM_FALSE;

        }

        return RM_FALSE;
    }


    /* RM_SLIP_RCV_COMPLETE */
    RM_RcvTimeoutCnt = 0;

    return RM_TRUE;
}


static void ManageSendData( uint8_t mode, uint8_t hasResponse, uint8_t masterCount )
{
    uint8_t  crc;
    uint8_t  isAssert;
    uint16_t result;
    uint8_t  data_size;
    uint16_t index;
    uint8_t* pData;
    uint8_t  frame_size;

    isAssert = CtrlTransmitTiming( mode, hasResponse );

    if( RM_IsTransmitPending == RM_TRUE )
    {
        RM_IsTransmitPending = RM_FALSE;
        isAssert = RM_TRUE;
    }

    if( RM_IsSndBusy == RM_TRUE )
    {
        if( isAssert == RM_TRUE )
        {
            RM_IsTransmitPending = RM_TRUE;
            isAssert = RM_FALSE;
        }

    }

    if( isAssert == RM_FALSE )
    {
        return;
    }
    else
    {
        if( mode == RM_LOG_MODE )
        {
            result = GetLogData( &RM_SndBufferArray[RM_FRAME_PAYLOAD], &data_size );
        }
        else
        {
            result = GetBlockData( &RM_SndBufferArray[RM_FRAME_PAYLOAD], &data_size );
        }

        if( result != RM_STATUS_ERR )
        {
            /* response opcode */
            RM_SndBufferArray[RM_FRAME_SEQCODE] = masterCount + RM_SlvCnt;
            RM_SlvCnt++;

            if( RM_SlvCnt > 0x0F )
            {
                RM_SlvCnt = 0x01;
            }

            frame_size = 1;

            /* payload(data) */
            /* Data is already stored in GetLogData() or GetBlockData() */
            frame_size += data_size;

            /* payload(crc) */
            crc = GetCRC(RM_SndBufferArray, frame_size);
            RM_SndBufferArray[frame_size] = crc;
            frame_size++;

            RM_AttachTransmitBuffer(RM_SndBufferArray, frame_size);

        }
    }
}


static uint8_t AnalyzeReceivedFrame(uint8_t opcode, uint8_t payload[], uint8_t payloadSize, uint8_t* mode)
{
    uint16_t result;

    result = RM_STATUS_ERR;

    if( RM_IsApproved != RM_TRUE )
    {
        if( opcode == 0x06 )
        {
            /* Try connecting */
            result = ValidatePassKey( payload, payloadSize );

            if( result != RM_STATUS_ERR )
            {
                RM_IsApproved = RM_TRUE;
            }

        }

    }
    else
    {
        switch( opcode )
        {
        case 0x01:
            /* Start Log mode */
            result = SetLogStart( payload, payloadSize );
            *mode = RM_LOG_MODE;
            break;

        case 0x02:
            /* Stop Log mode */
            result = SetLogStop( payload, payloadSize );
            *mode = RM_CANDR_MODE;
            break;

        case 0x03:
            /* Set Timing */
            result = SetTiming( payload, payloadSize );
            break;

        case 0x04:
            /* Write Value */
            result = WriteData( payload, payloadSize );
            break;

        case 0x05:
            /* Set Address and size for Log mode */
            result = SetLogData( payload, payloadSize );
            *mode = RM_CANDR_MODE;
            break;

        case 0x06:
            /* Read Information */
            result = ValidatePassKey( payload, payloadSize );
            *mode = RM_CANDR_MODE;
            break;

        case 0x07:
            /* Request Memory Dump */
            result = SetDumpData( payload, payloadSize );
            *mode = RM_CANDR_MODE;
            break;

        case 0x08:
            /* Pass Function to Another */
            result = SetBypassFunction( payload, payloadSize );
            *mode = RM_CANDR_MODE;
            break;

        default:
            break;
        }

    }

    if( result != RM_STATUS_ERR )
    {
        return RM_TRUE;
    }
    else
    {
        return RM_FALSE;
    }

}


static uint8_t CtrlTransmitTiming( uint8_t mode, uint8_t hasResponse )
{
    if( mode != RM_LOG_MODE )
    {
        RM_ReqTimeoutCnt = 0;
        RM_WaitCnt = 0;
        return hasResponse;
    }

    if( RM_TimingCnt == 0 )
    {
        RM_ReqTimeoutCnt = 0;
        RM_WaitCnt = 0;
        return hasResponse;
    }

    if( hasResponse == RM_TRUE )
    {
        RM_ReqTimeoutCnt = 0;
    }

    RM_ReqTimeoutCnt += RM_BaseCycle;
    if( RM_ReqTimeoutCnt >= RM_REQ_TIMEOUT_CNT )
    {
        /* Stop Interval transmission */
        RM_ReqTimeoutCnt = RM_REQ_TIMEOUT_CNT;
        RM_WaitCnt = 0;
        return RM_FALSE;
    }

    RM_WaitCnt += RM_BaseCycle;
    if( RM_WaitCnt >= RM_TimingCnt )
    {
        RM_WaitCnt = 0;
        return RM_TRUE;
    }
    else
    {
        return RM_FALSE;
    }

}


static uint16_t SetLogStart( uint8_t payload[], uint8_t payloadSize )
{
    if( payloadSize != 0 )
    {
        return RM_STATUS_ERR;
    }

    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_SUCCESS;
}


static uint16_t SetLogStop( uint8_t payload[], uint8_t payloadSize )
{
    if( payloadSize != 0 )
    {
        return RM_STATUS_ERR;
    }

    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_SUCCESS;
}


static uint16_t SetTiming( uint8_t payload[], uint8_t payloadSize )
{
    uint16_t timing;

    if( payloadSize != 2 )
    {
        return RM_STATUS_ERR;
    }
    else
    {
        timing  = (uint16_t)payload[1];
        timing  = timing << 8;
        timing |= (uint16_t)payload[0];

        RM_TimingCnt = timing;
    }

    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_SUCCESS;
}


static uint16_t WriteData( uint8_t payload[], uint8_t payloadSize )
{
#ifdef RM_ADDRESS_4BYTE
#define TABLE_SIZE   14
static const uint8_t RM_WriteContentsParser[TABLE_SIZE] =
{
    0,                      /* imaginary-padding */
    0, 0, 0, 0, 0,          /* size(1) and address(4) */
    1, 2, 0, 4, 0, 0, 0, 8  /* data length */
};

    uint32_t address;

#else
#define TABLE_SIZE   12
static const uint8_t RM_WriteContentsParser[TABLE_SIZE] =
{
    0,                      /* imaginary-padding */
    0, 0, 0,                /* size(1) and address(2) */
    1, 2, 0, 4, 0, 0, 0, 8  /* data length */
};

    uint16_t address;

#endif

    uint16_t index;

    uint8_t size;

    uint8_t  data8bit;
    uint8_t*  p_8bitAddr;

    uint16_t data16bit;
    uint16_t* p_16bitAddr;

    uint32_t data32bit;
    uint32_t* p_32bitAddr;

#ifdef RM_SUPPORT_64BIT
    uint64_t data64bit;
    uint64_t* p_64bitAddr;
#endif

    if( (payloadSize > TABLE_SIZE) ||
        (payloadSize == 0 ) )
    {
        return RM_STATUS_ERR;
    }

    size = payload[0];
    if( (size == 0) ||
        (size != RM_WriteContentsParser[payloadSize]) )
    {
        return RM_STATUS_ERR;
    }

#ifdef RM_ADDRESS_4BYTE
    address  = (uint32_t)payload[4];
    address  = address << 8;
    address |= (uint32_t)payload[3];
    address  = address << 8;
    address |= (uint32_t)payload[2];
    address  = address << 8;
    address |= (uint32_t)payload[1];

    /* set next index */
    index = 5;

#else
    address  = (uint16_t)payload[2];
    address  = address << 8;
    address |= (uint16_t)payload[1];

    /* set next index */
    index = 3;

#endif

    switch( size )
    {
    case 1:
        data8bit = payload[index];

        p_8bitAddr = (uint8_t *)address;
        *p_8bitAddr = data8bit;

        break;

    case 2:
        data16bit  = (uint16_t)payload[index+1];
        data16bit  = data16bit << 8;
        data16bit |= (uint16_t)payload[index];

        p_16bitAddr = (uint16_t *)address;
        *p_16bitAddr = data16bit;

        break;

    case 4:
        data32bit  = (uint32_t)payload[index+3];
        data32bit  = data32bit << 8;
        data32bit |= (uint32_t)payload[index+2];
        data32bit  = data32bit << 8;
        data32bit |= (uint32_t)payload[index+1];
        data32bit  = data32bit << 8;
        data32bit |= (uint32_t)payload[index];

        p_32bitAddr = (uint32_t *)address;
        *p_32bitAddr = data32bit;

        break;

#ifdef RM_SUPPORT_64BIT
    case 8:
        data64bit  = (uint64_t)payload[index+7];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+6];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+5];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+4];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+3];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+2];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index+1];
        data64bit  = data64bit << 8;
        data64bit |= (uint64_t)payload[index];

        p_64bitAddr = (uint64_t *)address;
        *p_64bitAddr = data64bit;

        break;
#endif

    default:
        break;

    }
    
    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_SUCCESS;

#undef TABLE_SIZE

}


static uint16_t SetLogData( uint8_t payload[], uint8_t payloadSize )
{
static uint8_t LogCurrentIndex;

#ifdef RM_ADDRESS_4BYTE
#define TABLE_SIZE      22
#define DATA_UNIT_NUM   5 /* size(1)+address(4) = 5 */
static const uint8_t RM_LogContentsParser[TABLE_SIZE] =
{
    0,                              /* imaginary-padding */
    0,                              /* code */
    0, 0, 0, 0, 1, 0, 0, 0, 0, 2,   /* (size(1) and address(4)) * 2 */
    0, 0, 0, 0, 3, 0, 0, 0, 0, 4    /* (size(1) and address(4)) * 2 */
};

    uint32_t address;

#else
#define TABLE_SIZE      26
#define DATA_UNIT_NUM   3 /* size(1)+address(2) = 3 */
static const uint8_t RM_LogContentsParser[TABLE_SIZE] =
{
    0,                  /* imaginary-padding */
    0,                  /* code */
    0, 0, 1, 0, 0, 2,   /* (size(1) and address(2)) * 2 */
    0, 0, 3, 0, 0, 4,   /* (size(1) and address(2)) * 2 */
    0, 0, 5, 0, 0, 6,   /* (size(1) and address(2)) * 2 */
    0, 0, 7, 0, 0, 8    /* (size(1) and address(2)) * 2 */
};

    uint16_t address;

#endif
    uint8_t  size;
    uint16_t total_size;
    uint16_t base_index;
    uint16_t index;
    uint8_t  bitmap;
    uint16_t max_index;

    if( (payloadSize > TABLE_SIZE) ||
        (payloadSize == 0 ) )
    {
        goto LABEL_SETLOG_FAILED;
    }

    max_index = (uint16_t)RM_LogContentsParser[payloadSize];

    if( max_index == 0 )
    {
        goto LABEL_SETLOG_FAILED;
    }

    bitmap = payload[0] & SETLOG_BIT_MASK;

    /* Detect Start of SetLogDataFrame */
    if( (bitmap & SETLOG_START_BIT ) == SETLOG_START_BIT )
    {
        LogCurrentIndex = 0;
    }

    if( (LogCurrentIndex + max_index) > RM_LOG_FACTOR_MAX )
    {
        goto LABEL_SETLOG_FAILED;
    }

    for( index = 0; index < max_index; index++ )
    {
        base_index = index * DATA_UNIT_NUM;
        base_index++;

        size = payload[base_index];

#ifdef RM_SUPPORT_64BIT
        if( (size != 1) && (size != 2) && (size != 4) && (size != 8) )
#else
        if( (size != 1) && (size != 2) && (size != 4) )
#endif
        {
            goto LABEL_SETLOG_FAILED;
        }

        RM_LogSizeArray[LogCurrentIndex] = size;

#ifdef RM_ADDRESS_4BYTE
        address  = (uint32_t)payload[base_index+4];
        address  = address << 8;
        address |= (uint32_t)payload[base_index+3];
        address  = address << 8;
        address |= (uint32_t)payload[base_index+2];
        address  = address << 8;
        address |= (uint32_t)payload[base_index+1];

#else
        address  = (uint16_t)payload[base_index+2];
        address  = address << 8;
        address |= (uint16_t)payload[base_index+1];

#endif
        RM_LogAddressArray[LogCurrentIndex] = address;
        LogCurrentIndex++;
    }

    /* Detect End of SetLogDataFrame */
    if( (bitmap & SETLOG_END_BIT ) == SETLOG_END_BIT )
    {
        RM_LogAvailableIndex = LogCurrentIndex;
    }
    
    total_size = 0;
    for( index = 0; index < LogCurrentIndex; index++ )
    {
        total_size += (uint16_t)RM_LogSizeArray[index];
    }

    if( total_size > RM_SND_PAYLOAD_SIZE )
    {
        goto LABEL_SETLOG_FAILED;
    }

LABEL_SETLOG_SUCCESS:
    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_SUCCESS;

LABEL_SETLOG_FAILED:
    LogCurrentIndex = 0;
    RM_LogAvailableIndex = 0;
    /* Return payload(data) size is zero */
    RM_BlockLength = 0;
    return RM_STATUS_ERR;

#undef TABLE_SIZE
#undef DATA_UNIT_NUM

}


static uint16_t ValidatePassKey( uint8_t payload[], uint8_t payloadSize )
{
    uint32_t passkey;

    if( payloadSize != 4 )
    {
        return RM_STATUS_ERR;
    }

    passkey  = payload[3];
    passkey  = passkey << 8;
    passkey |= payload[2];
    passkey  = passkey << 8;
    passkey |= payload[1];
    passkey  = passkey << 8;
    passkey |= payload[0];

    if( passkey != RM_PassKey )
    {
        return RM_STATUS_ERR;
    }

    RM_BlockAddress = RM_VersionAddress;
    RM_BlockLength = RM_VersionLength;
    return RM_STATUS_SUCCESS;
}


static uint16_t SetDumpData( uint8_t payload[], uint8_t payloadSize )
{
#ifdef RM_ADDRESS_4BYTE
#define ACCEPTABLE_SIZE 5

    uint32_t address;
#else
#define ACCEPTABLE_SIZE 3

    uint16_t address;
#endif

    if( payloadSize != ACCEPTABLE_SIZE )
    {
        return RM_STATUS_ERR;
    }

#ifdef RM_ADDRESS_4BYTE
    RM_BlockLength = payload[4];

    address  = (uint32_t)payload[3];
    address  = address << 8;
    address |= (uint32_t)payload[2];
    address  = address << 8;
    address |= (uint32_t)payload[1];
    address  = address << 8;
    address |= payload[0];

#else
    RM_BlockLength = payload[2];

    address  = (uint16_t)payload[1];
    address  = address << 8;
    address |= payload[0];

#endif

    RM_BlockAddress = address;

    return RM_STATUS_SUCCESS;

#undef ACCEPTABLE_SIZE
}


static  uint16_t  SetBypassFunction( uint8_t paylode[], uint8_t payloadSize )
{
    uint16_t result;
    RM_BYPASS_RESPONSE response;

    result = RM_STATUS_ERR;
    RM_BlockLength = 0;

    if( RM_BypassFunctionPtr != RM_FUNC_NULL )
    {
        response = RM_BypassFunctionPtr( paylode, payloadSize );

        RM_BlockAddress = response.Address;
        RM_BlockLength = response.Length;

        if(RM_BlockLength != 0)
        {
            result = RM_STATUS_SUCCESS;
        }

    }

    return result;
}


static uint16_t GetLogData( uint8_t payload[], uint8_t* payloadSize )
{
#ifdef RM_ADDRESS_4BYTE
    uint32_t address;
#else
    uint16_t address;
#endif
    uint16_t index;
    uint16_t payloadIndex;

    uint16_t totalSize;

    uint8_t  data8bit;
    uint8_t*  p_8bitAddr;

    uint16_t data16bit;
    uint16_t* p_16bitAddr;

    uint32_t data32bit;
    uint32_t* p_32bitAddr;

#ifdef RM_SUPPORT_64BIT
    uint64_t data64bit;
    uint64_t* p_64bitAddr;
#endif


    if(RM_LogAvailableIndex == 0)
    {
        return RM_STATUS_ERR;
    }

#if 0
    totalSize = 0;
    for( index = 0; index < RM_LogAvailableIndex; index++ )
    {
        totalSize += (uint16_t)RM_LogSizeArray[index];
    }

    if( totalSize > RM_SND_PAYLOAD_SIZE )
    {
        return RM_STATUS_ERR;
    }
#endif

    payloadIndex = 0;
    for( index = 0; index < (uint16_t)RM_LogAvailableIndex; index++ )
    {
#ifdef RM_ADDRESS_4BYTE
        address = RM_LogAddressArray[index];
#else
        address = RM_LogAddressArray[index];
#endif

        switch( RM_LogSizeArray[index] )
        {
        case 1:
            p_8bitAddr = (uint8_t *)address;

            data8bit = *p_8bitAddr;

            payload[payloadIndex] = data8bit;
            payloadIndex++;

            break;

        case 2:
            p_16bitAddr = (uint16_t *)address;

            data16bit = *p_16bitAddr;

            payload[payloadIndex] = (uint8_t)(data16bit);
            payloadIndex++;
            data16bit = data16bit >> 8;
            payload[payloadIndex] = (uint8_t)(data16bit);
            payloadIndex++;

            break;

        case 4:
            p_32bitAddr = (uint32_t *)address;

            data32bit = *p_32bitAddr;

            payload[payloadIndex] = (uint8_t)(data32bit);
            payloadIndex++;
            data32bit = data32bit >> 8;
            payload[payloadIndex] = (uint8_t)(data32bit);
            payloadIndex++;
            data32bit = data32bit >> 8;
            payload[payloadIndex] = (uint8_t)(data32bit);
            payloadIndex++;
            data32bit = data32bit >> 8;
            payload[payloadIndex] = (uint8_t)(data32bit);
            payloadIndex++;

            break;

#ifdef RM_SUPPORT_64BIT
        case 8:
            p_64bitAddr = (uint64_t *)address;

            data64bit = *p_64bitAddr;

            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;
            data64bit = data64bit >> 8;
            payload[payloadIndex] = (uint8_t)(data64bit);
            payloadIndex++;

            break;

#endif
        default:
            break;

        }

    }

    *payloadSize = (uint8_t)payloadIndex;

    return RM_STATUS_SUCCESS;
}


static uint16_t GetBlockData( uint8_t payload[], uint8_t* payloadSize )
{
#ifdef RM_ADDRESS_4BYTE
    uint32_t address;
#else
    uint16_t address;
#endif
    uint16_t index;

    uint8_t* paddr;
    uint8_t  abyte;

    uint16_t result;

#ifdef RM_ADDRESS_4BYTE
    address = RM_BlockAddress;
#else
    address = RM_BlockAddress;
#endif

    result = RM_STATUS_SUCCESS;

    paddr = (uint8_t *)address;
    for( index = 0; index < (uint16_t)RM_BlockLength; index++ )
    {
        abyte = *paddr;
        if( index > RM_SND_PAYLOAD_SIZE )
        {
            result = RM_STATUS_ERR;
            break;
        }
        else
        {
            payload[index] = abyte;
            paddr++;
        }

    }

    *payloadSize = (uint8_t)index;

    return result;
}


static uint8_t GetCRC( uint8_t buffer[], uint8_t bufferSize )
{
/* x^8+x^7+x^4+x^2+1 */
static const uint8_t RM_CrcTable[256] =
{
 /*      0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F         */
      0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d /* 0x00 */
    , 0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f /* 0x10 */
    , 0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9 /* 0x20 */
    , 0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b /* 0x30 */
    , 0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0 /* 0x40 */
    , 0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2 /* 0x50 */
    , 0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44 /* 0x60 */
    , 0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16 /* 0x70 */
    , 0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92 /* 0x80 */
    , 0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0 /* 0x90 */
    , 0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36 /* 0xA0 */
    , 0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64 /* 0xB0 */
    , 0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f /* 0xC0 */
    , 0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d /* 0xD0 */
    , 0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab /* 0xE0 */
    , 0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9 /* 0xF0 */
};

    uint8_t index;
    uint8_t crcIndex;
    uint8_t crc;

    crc = 0;

    for( index = 0; index < bufferSize; index++ )
    {
      crcIndex = crc ^ buffer[ index ];
        crc = RM_CrcTable[ crcIndex ];
    }

    return crc;

}


void RM_Initialize( uint8_t version[], uint8_t versionSize, uint8_t base_cycle, uint32_t passkey )
{
#ifdef RM_ADDRESS_4BYTE
    RM_VersionAddress = (uint32_t)version;
#else
    RM_VersionAddress = (uint16_t)version;
#endif
    RM_VersionLength = versionSize;

    if( base_cycle > 0 )
    {
        RM_BaseCycle = base_cycle;
    }
    else
    {
        RM_BaseCycle = RM_DEFAULT_BASE_CYCLE;
    }

    RM_PassKey = passkey;

    RM_LogAvailableIndex = 0;

    RM_RcvTimeoutCnt = 0;
    RM_ReqTimeoutCnt = 0;
    RM_WaitCnt = RM_BaseCycle;
    RM_TimingCnt = RM_SND_DEFAULT_CNT;

    RM_RcvBufferSize = 0;
    RM_SndBufferSize = 0;

    RM_SndBufferSentIndex = 0;

    RM_MasCnt = 0x00;
    RM_SlvCnt = 0x01; /* initial slv_cnt is "0xX1" */

    RM_CommMode = RM_INITIAL_MODE;

    RM_RcvRingBuffEndIndex = 0;
    RM_RcvRingBuffBgnIndex = 0;
    RM_IsRcvSLIPActive = RM_FALSE;

    RM_IsSndSLIPActive = RM_FALSE;
    RM_IsSndBusy = RM_FALSE;

    RM_IsApproved = RM_FALSE;

    RM_IsTransmitPending = RM_FALSE;

    RM_BypassFunctionPtr = RM_FUNC_NULL;

}


void RM_AttachBypassFunction( RM_FUNCPTR func )
{
    RM_BypassFunctionPtr = func;
}


void RM_AttachTransmitBuffer(uint8_t tmp[], uint8_t size)
{
    if(RM_IsSndBusy == RM_TRUE)
        return;

    RM_IsSndBusy = RM_TRUE;
    RM_SndBufferSentIndex = 0;
    RM_SndBufferSize = size;
    RM_SndBufferPtr = tmp;
}


uint8_t* RM_TryTransmission( void )
{
    uint8_t* pbyte;

    if( RM_IsSndSLIPActive == RM_FALSE )
    {
        pbyte = RM_GetTransmitData();

    }
    else
    {
        pbyte = RM_DATA_NULL;

    }

    return pbyte;

}


uint8_t* RM_GetTransmitData( void )
{
static uint8_t SndSLIPTmpData;
static uint8_t IsSndSLIPeof;
static uint8_t SndSLIPEscChar;
static uint8_t SndSLIPLastChar;

    uint8_t* pbyte;

    pbyte = RM_DATA_NULL;

    if( RM_SndBufferSentIndex < RM_SndBufferSize )
    {
        if( RM_IsSndSLIPActive == RM_FALSE )
        {
            SndSLIPLastChar = 0x00;
            IsSndSLIPeof = RM_FALSE;
            RM_IsSndSLIPActive = RM_TRUE;

            SndSLIPTmpData = RM_FRAME_CHAR_END;
            pbyte = &SndSLIPTmpData;
        }
        else
        {
            if( SndSLIPLastChar == RM_FRAME_CHAR_ESC )
            {
                SndSLIPTmpData = SndSLIPEscChar;
            }
            else
            {
                SndSLIPTmpData = RM_SndBufferPtr[ RM_SndBufferSentIndex ];
            }

            if( SndSLIPTmpData == RM_FRAME_CHAR_END )
            {
                SndSLIPTmpData = RM_FRAME_CHAR_ESC;
                SndSLIPEscChar = RM_FRAME_CHAR_ESC_END;
            }
            else if( SndSLIPTmpData == RM_FRAME_CHAR_ESC )
            {
                SndSLIPTmpData = RM_FRAME_CHAR_ESC;
                SndSLIPEscChar = RM_FRAME_CHAR_ESC_ESC;
            }
            else
            {
                RM_SndBufferSentIndex++;
            }

            SndSLIPLastChar = SndSLIPTmpData;
            pbyte = &SndSLIPTmpData;
        }

    }
    else
    {
        if( RM_IsSndSLIPActive == RM_TRUE )
        {
            if( IsSndSLIPeof == RM_FALSE )
            {
                IsSndSLIPeof = RM_TRUE;
                SndSLIPTmpData = RM_FRAME_CHAR_END;
                pbyte = &SndSLIPTmpData;
            }
            else
            {
                IsSndSLIPeof = RM_FALSE;
                RM_IsSndSLIPActive = RM_FALSE;
                RM_IsSndBusy = RM_FALSE;
            }

        }

    }

    return pbyte;

}


uint16_t RM_SetReceivedData( uint8_t abyte )
{
    uint16_t result;

    result = RM_BUFFER_FULL;

    if( ((RM_RcvRingBuffEndIndex + 1) & RM_UART_BUFF_MASK ) != RM_RcvRingBuffBgnIndex )
    {
        RM_RcvRingBuffArray[ RM_RcvRingBuffEndIndex ] = abyte;
        RM_RcvRingBuffEndIndex = (RM_RcvRingBuffEndIndex + 1) & RM_UART_BUFF_MASK;

        result = RM_BUFFER_OK;
    }

    return result;
}


static uint16_t GetReceivedFrame( uint8_t rcvBuffer[], uint8_t *rcvdLength, uint8_t maxLength )
{
static uint8_t RcvSLIPLastChar;

    uint8_t  data;
    uint8_t  size;
    uint8_t  crc;
    uint16_t result;

    if( RM_IsRcvSLIPActive == RM_FALSE )
    {
        result = RM_SLIP_RCV_NOTHING;
    }
    else
    {
        result = RM_SLIP_RCV_INCOMING;
    }

    size = (RM_RcvRingBuffEndIndex - RM_RcvRingBuffBgnIndex) & RM_UART_BUFF_MASK;
    while(size != 0)
    {
        if(*rcvdLength >= maxLength)
            goto LABEL_RCVDFRAME_FAILED;

        data = RM_RcvRingBuffArray[ RM_RcvRingBuffBgnIndex ];
        RM_RcvRingBuffBgnIndex = (RM_RcvRingBuffBgnIndex + 1) & RM_UART_BUFF_MASK;

        size--;

        if( data == RM_FRAME_CHAR_END )
        {
            if( RM_IsRcvSLIPActive == RM_FALSE )
            {
                /* Start SLIP Frame */
                RM_IsRcvSLIPActive = RM_TRUE;
                result = RM_SLIP_RCV_INCOMING;
                RcvSLIPLastChar = 0x00;
                continue;
            }

            if( *rcvdLength == 0 )
            {
                /* receive order RM_FRAME_CHAR_END,RM_FRAME_CHAR_END */
                /* Last RM_FRAME_CHAR_END might be Start of Frame */
                continue;
            }

            crc = GetCRC(rcvBuffer, *rcvdLength);
            if( crc != 0 )
            {
                /* Purge received data */
                *rcvdLength = 0;
                continue;
            }

            /* End SLIP Frame */
            RM_IsRcvSLIPActive = RM_FALSE;
            result = RM_SLIP_RCV_COMPLETE;
            (*rcvdLength)--;     // delete useless crc data size
            break;

        }
        else
        {
            if( RM_IsRcvSLIPActive == RM_FALSE )
            {
                /* Ignore received data */
                continue;
            }

            /* payload of frame */
            if( RcvSLIPLastChar == RM_FRAME_CHAR_ESC )
            {
                if( data == RM_FRAME_CHAR_ESC_END )
                {
                    rcvBuffer[*rcvdLength] = RM_FRAME_CHAR_END;
                    (*rcvdLength)++;
                }
                else if( data == RM_FRAME_CHAR_ESC_ESC )
                {
                    rcvBuffer[*rcvdLength] = RM_FRAME_CHAR_ESC;
                    (*rcvdLength)++;
                }
                else
                {
                    goto LABEL_RCVDFRAME_FAILED;
                }
            }
            else
            {
                if( data == RM_FRAME_CHAR_ESC )
                {
                    /* Start escape sequence */
                }
                else
                {
                    rcvBuffer[*rcvdLength] = data;
                    (*rcvdLength)++;
                }
            }

            if( *rcvdLength >= RM_RCV_FRAME_BUFF_SIZE )
            {
                goto LABEL_RCVDFRAME_FAILED;
            }

        }

        RcvSLIPLastChar = data;
    }

    return result;

LABEL_RCVDFRAME_FAILED:
    RM_IsRcvSLIPActive = RM_FALSE;
    *rcvdLength = 0;
    return RM_SLIP_RCV_FAULT;
}


uint8_t RM_IsTransmissionBusy()
{
    return RM_IsSndBusy;
}


uint8_t RM_Connected()
{
    if( RM_CommMode == RM_LOG_MODE &&
        RM_IsApproved == RM_TRUE )
    {
        return RM_TRUE;
    }
    else
    {
        return RM_FALSE;
    }

}


/*-- end of file --*/
