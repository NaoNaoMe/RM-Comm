/*
 * RMonitor.h
 *
 *  Created on: 2019/06/04
 *      Author: NaoyaImai
 */
#pragma once

#ifndef RMonitor_H_
#define RMonitor_H_
#include <Arduino.h>
#include "RmComm.h"

#define TEXT_RX_BUFFER_SIZE 128
#define TEXT_TX_BUFFER_SIZE 64

#define MONITOR_RX_BUFFER_SIZE 128
#define MONITOR_TX_BUFFER_SIZE 16

class RMonitor : public Print{
public:
    RMonitor();

    void initialize(uint8_t version[], uint8_t v_length, uint8_t base_cycle, uint32_t password);

    void task();

    void attachBypassFunction( RM_FUNCPTR func );

    int available();
    int peek();
    int read();
    size_t write(const uint8_t data);
    using Print::write; // pull in write(str) and write(buf, size) from Print

private:
    volatile uint8_t rx_buffer_head;
    volatile uint8_t rx_buffer_tail;
    volatile uint8_t tx_buffer_head;
    volatile uint8_t tx_buffer_tail;

    unsigned char rx_buffer[TEXT_RX_BUFFER_SIZE];
    unsigned char tx_buffer[TEXT_TX_BUFFER_SIZE];

    uint8_t rcvBufferArray[MONITOR_RX_BUFFER_SIZE];
    uint8_t rcvBufferSize;

    uint8_t sndBufferArray[MONITOR_TX_BUFFER_SIZE];

    void rx_write(uint8_t c);
    int  tx_available();
    int  tx_read();

};


#endif /* RMonitor_H_ */
