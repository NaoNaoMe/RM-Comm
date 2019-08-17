/*
 * RMonitor.cpp
 *
 *  Created on: 2019/06/04
 *      Author: NaoyaImai
 */
#include "RMonitor.h"


RMonitor::RMonitor()
{
}


void RMonitor::initialize(uint8_t version[], uint8_t v_length, uint8_t base_cycle, uint32_t password)
{
    RM_Initialize( version, v_length, base_cycle, password );

    rx_buffer_head = 0;
    rx_buffer_tail = 0;
    tx_buffer_head = 0;
    tx_buffer_tail = 0;

    rcvBufferSize = 0;
}


void RMonitor::task()
{
    while (Serial.available())
    {
      uint8_t data = Serial.read();

      RM_SetReceivedData(data);

    }

    uint8_t isSuccess;
    isSuccess = RM_DecodeRcvdData( rcvBufferArray, &rcvBufferSize, MONITOR_RX_BUFFER_SIZE );

    if(isSuccess == RM_TRUE)
    {
        if(rcvBufferArray[0] == 0x00)
        {
            isSuccess = RM_FALSE;

            if(rcvBufferArray[1] == 0x01)
            {
                //Umanged data coming
                for(int i = 2; i < rcvBufferSize; i++)
                    rx_write(rcvBufferArray[i]);
            }

        }

    }

    RM_TaskCore( isSuccess, rcvBufferArray, &rcvBufferSize );

    if(RM_Connected() == RM_TRUE &&
       RM_IsTransmissionBusy() == RM_FALSE &&
       tx_available() != 0)
    {
        /* response opcode */
        sndBufferArray[0] = 0;  // declare Unmanaged data frame
        sndBufferArray[1] = 1;  // text data

        /* payload(data) */
        int i;
        for(i = 2; i < MONITOR_TX_BUFFER_SIZE; i++)
        {
            if(tx_available() == 0)
                break;
            else
                sndBufferArray[i] = tx_read();
        }

        RM_AttachTransmitBuffer(sndBufferArray, (uint8_t)i);

    }

    uint8_t *pData = RM_TryTransmission();

    while( pData != RM_DATA_NULL )
    {
        Serial.write(*pData);
        pData = RM_GetTransmitData();
    }

}


void RMonitor::attachBypassFunction( RM_FUNCPTR func )
{
    RM_AttachBypassFunction( func );
}


int RMonitor::available()
{
    return ((unsigned int)(TEXT_RX_BUFFER_SIZE + rx_buffer_head - rx_buffer_tail)) % TEXT_RX_BUFFER_SIZE;
}

int RMonitor::peek()
{
    if (rx_buffer_head == rx_buffer_tail)
    {
        return -1;
    }
    else
    {
        return rx_buffer[rx_buffer_tail];
    }

}

int RMonitor::read()
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (rx_buffer_head == rx_buffer_tail)
    {
        return -1;
    }
    else
    {
        uint8_t c = rx_buffer[rx_buffer_tail];
        rx_buffer_tail = (uint8_t)(rx_buffer_tail + 1) % TEXT_RX_BUFFER_SIZE;
        return c;
    }

}

void RMonitor::rx_write(uint8_t c)
{
    uint8_t i = (uint8_t)(rx_buffer_head + 1) % TEXT_RX_BUFFER_SIZE;

    if (i != rx_buffer_tail) {
      rx_buffer[rx_buffer_head] = c;
      rx_buffer_head = i;
    }
}

int RMonitor::tx_available()
{
    return ((unsigned int)(TEXT_TX_BUFFER_SIZE + tx_buffer_head - tx_buffer_tail)) % TEXT_TX_BUFFER_SIZE;
}

int RMonitor::tx_read()
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (tx_buffer_head == tx_buffer_tail)
    {
        return -1;
    }
    else
    {
        uint8_t c = tx_buffer[tx_buffer_tail];
        tx_buffer_tail = (uint8_t)(tx_buffer_tail + 1) % TEXT_TX_BUFFER_SIZE;
        return c;
    }

}

size_t RMonitor::write(const uint8_t c)
{
    uint8_t i = (uint8_t)(tx_buffer_head + 1) % TEXT_TX_BUFFER_SIZE;

    if (i != tx_buffer_tail) {
      tx_buffer[tx_buffer_head] = c;
      tx_buffer_head = i;
    }

    return 1;
}
