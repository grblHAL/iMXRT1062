/*
  uart.c - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of grblHAL

  Some parts of this code is Copyright (c) 2020-2021 Terje Io

  Some parts are derived from HardwareSerial.cpp in the Teensyduino Core Library

*/

/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string.h>

#include "driver.h"
#include "grbl/protocol.h"

#define UART_CLOCK 24000000
#define CTRL_ENABLE         (LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE)
#define CTRL_TX_ACTIVE      (CTRL_ENABLE | LPUART_CTRL_TIE)
#define CTRL_TX_COMPLETING  (CTRL_ENABLE | LPUART_CTRL_TCIE)
#define CTRL_TX_INACTIVE    CTRL_ENABLE

#ifndef UART_PORT
#define UART uart1_hardware
#elif UART_PORT == 5
#define UART uart5_hardware
#else
#error "UART port not available!"
#endif

typedef struct {
    IMXRT_LPUART_t *port;
    volatile uint32_t *ccm_register;
    const uint32_t ccm_value;
    enum IRQ_NUMBER_t irq;
    void (*irq_handler)(void);
    pin_info_t rx_pin;
    pin_info_t tx_pin;
} uart_hardware_t;

static void uart_interrupt_handler (void);

static const uart_hardware_t uart1_hardware =
{
    .port = &IMXRT_LPUART6,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART6(CCM_CCGR_ON),
    .irq = IRQ_LPUART6,
    .irq_handler = uart_interrupt_handler,
    .rx_pin = {
        .pin = 0,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = 1,
        .mux_val = 2,
        .select_reg = NULL,
        .select_val = 0
    }
};

static const uart_hardware_t uart5_hardware =
{
    .port = &IMXRT_LPUART8,
    .ccm_register = &CCM_CCGR6,
    .ccm_value = CCM_CCGR6_LPUART8(CCM_CCGR_ON),
    .irq = IRQ_LPUART8,
    .irq_handler = uart_interrupt_handler,
    .rx_pin = {
        .pin = 21,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = 20,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT,
        .select_val = 1
    }
};

/*

#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, mask)         ((*((base)+2) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+1) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+1) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+34) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+33) = (mask))


static bool transmitting_ = 0;
volatile uint32_t   *transmit_pin_baseReg_ = 0;
uint32_t            transmit_pin_bitmask_ = 0;

void transmitterEnable(uint8_t pin)
{
    while (transmitting_) ;
    pinMode(pin, OUTPUT);
    transmit_pin_baseReg_ = PIN_TO_BASEREG(pin);
    transmit_pin_bitmask_ = PIN_TO_BITMASK(pin);
    DIRECT_WRITE_LOW(transmit_pin_baseReg_, transmit_pin_bitmask_);
}
*/

static uint16_t tx_fifo_size;
static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuffer.tail;         // Get buffer pointer

    if(tail == rxbuffer.head)
        return -1; // no data available

    int16_t data = rxbuffer.data[tail];         // Get next character, increment tmp pointer
    rxbuffer.tail = BUFNEXT(tail, rxbuffer);    // and update pointer

    return data;
}

static void serialTxFlush (void)
{
    txbuffer.tail = txbuffer.head;
}

static uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

static void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = BUFNEXT(rxbuffer.head, rxbuffer);
}

static bool serialPutC (const char c)
{
    if(txbuffer.head == txbuffer.tail && ((UART.port->WATER >> 8) & 0x7) < tx_fifo_size) {
        UART.port->DATA  = c;
        return true;
    }

    uint_fast16_t next_head = BUFNEXT(txbuffer.head, txbuffer);   // Get next head pointer

    while(txbuffer.tail == next_head) {             // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;               // Add data to buffer
    txbuffer.head = next_head;                      // and update head pointer

    __disable_irq();
    UART.port->CTRL |= LPUART_CTRL_TIE; // (may need to handle this issue)BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT); // Enable TX interrupts
    __enable_irq();

    return true;
}

static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

static uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART.port->WATER >> 8) & 0x7) + ((UART.port->STAT & LPUART_STAT_TC) ? 0 : 1);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
    float base = (float)UART_CLOCK / (float)baud_rate;
    float besterr = 1e20;
    int bestdiv = 1;
    int bestosr = 4;
    for (int osr = 4; osr <= 32; osr++) {
        float div = base / (float)osr;
        int divint = (int)(div + 0.5f);
        if (divint < 1)
            divint = 1;
        else if (divint > 8191)
            divint = 8191;
        float err = ((float)divint - div) / div;
        if (err < 0.0f)
            err = -err;
        if (err <= besterr) {
            besterr = err;
            bestdiv = divint;
            bestosr = osr;
        }
    }

    UART.port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv) | (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);

    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)
        NVIC_DISABLE_IRQ(UART.irq);
    else
        NVIC_ENABLE_IRQ(UART.irq);

    return true;
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    PROGMEM static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n = serialWrite,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    *UART.ccm_register |= UART.ccm_value;

    *(portControlRegister(UART.rx_pin.pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(UART.rx_pin.pin)) = UART.rx_pin.mux_val;
    if (UART.rx_pin.select_reg)
        *(UART.rx_pin.select_reg) = UART.rx_pin.select_val;

    *(portControlRegister(UART.tx_pin.pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(UART.tx_pin.pin)) = UART.tx_pin.mux_val;
    if (UART.tx_pin.select_reg)
        *(UART.tx_pin.select_reg) = UART.tx_pin.select_val;

    serialSetBaudRate(baud_rate);

    UART.port->PINCFG = 0;

    // Enable the transmitter, receiver and enable receiver interrupt
    NVIC_DISABLE_IRQ(UART.irq);
    attachInterruptVector(UART.irq, UART.irq_handler);
    NVIC_SET_PRIORITY(UART.irq, 0);
    NVIC_ENABLE_IRQ(UART.irq);

    tx_fifo_size = (UART.port->FIFO >> 4) & 0x7;
    tx_fifo_size = tx_fifo_size ? (2 << tx_fifo_size) : 1;

    uint8_t tx_water = (tx_fifo_size < 16) ? tx_fifo_size >> 1 : 7;
    uint16_t rx_fifo_size = (((UART.port->FIFO >> 0) & 0x7) << 2);
    uint8_t rx_water = (rx_fifo_size < 16) ? rx_fifo_size >> 1 : 7;
    /*
    Serial.printf("SerialX::begin stat:%x ctrl:%x fifo:%x water:%x\n", UART.port->STAT, UART.port->CTRL, UART.port->FIFO, UART.port->WATER );
    Serial.printf("  FIFO sizes: tx:%d rx:%d\n",tx_fifo_size, rx_fifo_size);
    Serial.printf("  Watermark tx:%d, rx: %d\n", tx_water, rx_water);
    */
    UART.port->WATER = LPUART_WATER_RXWATER(rx_water) | LPUART_WATER_TXWATER(tx_water);
    UART.port->FIFO |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;
    // lets configure up our CTRL register value
    uint32_t ctrl = CTRL_TX_INACTIVE;

uint16_t format = 0;

    // Now process the bits in the Format value passed in
    // Bits 0-2 - Parity plus 9  bit.
    ctrl |= (format & (LPUART_CTRL_PT | LPUART_CTRL_PE) );  // configure parity - turn off PT, PE, M and configure PT, PE
    if (format & 0x04)
        ctrl |= LPUART_CTRL_M;       // 9 bits (might include parity)
    if ((format & 0x0F) == 0x04)
        ctrl |=  LPUART_CTRL_R9T8; // 8N2 is 9 bit with 9th bit always 1

    // Bit 5 TXINVERT
    if (format & 0x20)
        ctrl |= LPUART_CTRL_TXINV;       // tx invert

    // write out computed CTRL
    UART.port->CTRL = ctrl;

    // Bit 3 10 bit - Will assume that begin already cleared it.
    // process some other bits which change other registers.
    if (format & 0x08)  UART.port->BAUD |= LPUART_BAUD_M10;

    // Bit 4 RXINVERT
    uint32_t c = UART.port->STAT & ~LPUART_STAT_RXINV;
    if (format & 0x10) c |= LPUART_STAT_RXINV;      // rx invert
    UART.port->STAT = c;

    // bit 8 can turn on 2 stop bit mode
    if (format & 0x100)
        UART.port->BAUD |= LPUART_BAUD_SBNS;

    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint32_t ctrl = UART.port->CTRL;

    if((ctrl & LPUART_CTRL_TIE) && (UART.port->STAT & LPUART_STAT_TDRE))
    {
        uint_fast16_t tail = txbuffer.tail;                 // Get buffer pointer

        do {
            if(txbuffer.head != tail) {
                UART.port->DATA = txbuffer.data[tail];      // Put character in TXT register
                tail = BUFNEXT(tail, txbuffer);             // and update tmp tail pointer
            } else
                break;
        } while(((UART.port->WATER >> 8) & 0x7) < tx_fifo_size);

        if((txbuffer.tail = tail) == txbuffer.head)  // Disable TX interrups
            UART.port->CTRL &= ~LPUART_CTRL_TIE;
    }

    if ((ctrl & LPUART_CTRL_TCIE) && (UART.port->STAT & LPUART_STAT_TC))
        UART.port->CTRL &= ~LPUART_CTRL_TCIE;

    if(UART.port->STAT & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) {

        while((UART.port->WATER >> 24) & 0x7) {
            uint32_t data = UART.port->DATA & 0xFF;                         // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rxbuffer.head, rxbuffer); // Get next head pointer
                if(next_head == rxbuffer.tail)                              // If buffer full
                    rxbuffer.overflow = true;                               // flag overflow
                else {
                    rxbuffer.data[rxbuffer.head] = (char)data;              // Add data to buffer
                    rxbuffer.head = next_head;                              // and update pointer
                }
            }
        }

        if (UART.port->STAT & LPUART_STAT_IDLE)
            UART.port->STAT |= LPUART_STAT_IDLE; // writing a 1 to idle should clear it.
    }
}
