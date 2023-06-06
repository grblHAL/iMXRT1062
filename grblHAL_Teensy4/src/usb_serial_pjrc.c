/*

  usb_serial_pjrc.c - driver code for IMXRT1062 processor (on Teensy 4.0 board) : USB serial port wrapper, PJRC version

  Part of grblHAL

  Copyright (c) 2018-2022 Terje Io


  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>

#include "usb_serial.h"

#include "driver.h"
#include "grbl/protocol.h"

#if USB_SERIAL_CDC == 2

#define BLOCK_RX_BUFFER_SIZE 20

static stream_block_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

extern volatile uint32_t usb_cdc_line_rtsdtr_millis;
extern volatile uint32_t systick_millis_count;
extern volatile uint8_t usb_cdc_line_rtsdtr;
extern volatile uint8_t usb_configuration;

static bool usb_isConnected (void)
{
    return usb_configuration && (usb_cdc_line_rtsdtr & USB_SERIAL_DTR) &&
            ((uint32_t)(systick_millis_count - usb_cdc_line_rtsdtr_millis) >= 15);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t usb_serialRxCount (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;
    return (uint16_t)BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t usb_serialRxFree (void)
{
    uint_fast16_t tail = rxbuf.tail, head = rxbuf.head;

    return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the serial input buffer (including the USB buffer)
//
static void usb_serialRxFlush (void)
{
    usb_serial_flush_input();
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void usb_serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = CMD_RESET;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool usb_serialPutC (const char c)
{
    usb_serial_putchar(c);

    return true;
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool _usb_write (void)
{
    size_t length, txfree;

    txbuf.s = txbuf.data;

    while(txbuf.length) {

        if((txfree = usb_serial_write_buffer_free()) > 10) {

            length = txfree < txbuf.length ? txfree : txbuf.length;

            usb_serial_write(txbuf.s, length); //

            txbuf.length -= length;
            txbuf.s += length;
        }

        if(txbuf.length && !hal.stream_blocking_callback()) {
            txbuf.length = 0;
            txbuf.s = txbuf.data;
            return false;
        }
    }

    txbuf.s = txbuf.data;

    return true;
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usb_serialWrite (const char *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!_usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!_usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        _usb_write();
    }
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void usb_serialWriteS (const char *s)
{
    if(*s == '\0')
        return;

    size_t length = strlen(s);

    if((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE) {

        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if(s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length) {
            if(!_usb_write())
                return;
        }
    } else
        usb_serialWrite(s, (uint16_t)length);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usb_serialGetC (void)
{
    if(rxbuf.tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[rxbuf.tail];         // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(rxbuf.tail, rxbuf);    // and update pointer

    return (int16_t)data;
}

static bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usb_serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usb_serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

//
// This function get called from the foregorund process,
// used here to get characters off the USB serial input stream and buffer
// them for processing by the core. Real time command characters are stripped out
// and submitted for realtime processing.
//
void usb_execute_realtime (sys_state_t state)
{
    static volatile bool lock = false;
    static volatile uint32_t last_micros = 0;
    static char tmpbuf[BLOCK_RX_BUFFER_SIZE];

    //if(lock)
    //    return;

    uint32_t current_micros;
    if(lock || ((current_micros = micros()) - last_micros) < 50)
        return;

    char c, *dp;
    int avail, free;

    lock = true;
    last_micros = current_micros;

    if((avail = usb_serial_available())) {

        dp = tmpbuf;
        free = usb_serialRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;

        avail = usb_serial_read(tmpbuf, avail > free ? free : avail);

        while(avail--) {
            c = *dp++;
            if(!enqueue_realtime_command(c)) {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get next head pointer
                if(next_head == rxbuf.tail)                             // If buffer full
                    rxbuf.overflow = On;                                // flag overflow,
                else {
                    rxbuf.data[rxbuf.head] = c;                         // else add character data to buffer
                    rxbuf.head = next_head;                             // and update pointer
                }
            }
        }
    }

    lock = false;
}

const io_stream_t *usb_serialInit (void)
{
    PROGMEM static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state = { .is_usb = On },
        .is_connected = usb_isConnected,
        .read = usb_serialGetC,
        .write = usb_serialWriteS,
        .write_char = usb_serialPutC,
        .write_n = usb_serialWrite,
        .enqueue_rt_command = usb_serialEnqueueRtCommand,
        .get_rx_buffer_free = usb_serialRxFree,
        .get_rx_buffer_count = usb_serialRxCount,
        .reset_read_buffer = usb_serialRxFlush,
        .cancel_read_buffer = usb_serialRxCancel,
        .suspend_read = usb_serialSuspendInput,
        .set_enqueue_rt_handler = usb_serialSetRtHandler
    };

//    usb_serial_configure(); // Done somewhere already - do not call again
    txbuf.s = txbuf.data;
    txbuf.max_length = usb_serial_write_buffer_free(); // 6144
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    grbl.on_execute_realtime = usb_execute_realtime;

    return &stream;
}


#endif
