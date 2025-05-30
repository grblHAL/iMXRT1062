/*
  uart.c - driver code for IMXRT1062 processor (on Teensy 4.x board)

  Part of grblHAL

  Some parts of this code is Copyright (c) 2020-2025 Terje Io
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
#include <stdbool.h>
#include <stdint.h>

#include "driver.h"
#include "uart.h"

#include "grbl/protocol.h"

#define UART_CLOCK 24000000
#define CTRL_ENABLE         (LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE)
#define CTRL_TX_ACTIVE      (CTRL_ENABLE | LPUART_CTRL_TIE)
#define CTRL_TX_COMPLETING  (CTRL_ENABLE | LPUART_CTRL_TCIE)
#define CTRL_TX_INACTIVE    CTRL_ENABLE

#define UART uart_hardware

typedef struct {
    IMXRT_LPUART_t *port;
    volatile uint32_t *ccm_register;
    const uint32_t ccm_value;
    enum IRQ_NUMBER_t irq;
    void (*irq_handler)(void);
    pin_info_t rx_pin;
    pin_info_t tx_pin;
} uart_hardware_t;

static uint16_t tx_fifo_size;
DMAMEM static stream_tx_buffer_t txbuffer;
DMAMEM static stream_rx_buffer_t rxbuffer;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static const io_stream_t *serialInit (uint32_t baud_rate);
static void uart_interrupt_handler (void);

#if SERIAL_PORT == 6

#define RX_PIN      0
#define TX_PIN      1
#define SERIAL_PORT 6

static const uart_hardware_t uart_hardware =
{
    .port = &IMXRT_LPUART6,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART6(CCM_CCGR_ON),
    .irq = IRQ_LPUART6,
    .irq_handler = uart_interrupt_handler,
    .rx_pin = {
        .pin = RX_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX_PIN,
        .mux_val = 2,
        .select_reg = NULL,
        .select_val = 0
    }
};

#elif SERIAL_PORT == 5

#define RX_PIN 34
#define TX_PIN 35

static const uart_hardware_t uart_hardware =
{
    .port = &IMXRT_LPUART5,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART5(CCM_CCGR_ON),
    .irq = IRQ_LPUART5,
    .irq_handler = uart_interrupt_handler,
    .rx_pin = {
        .pin = RX_PIN,
        .mux_val = 1,
        .select_reg = &IOMUXC_LPUART5_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX_PIN,
        .mux_val = 1,
        .select_reg = &IOMUXC_LPUART5_TX_SELECT_INPUT,
        .select_val = 1
    }
};

#elif SERIAL_PORT == 8

#define RX_PIN 21
#define TX_PIN 20

static const uart_hardware_t uart_hardware =
{
    .port = &IMXRT_LPUART8,
    .ccm_register = &CCM_CCGR6,
    .ccm_value = CCM_CCGR6_LPUART8(CCM_CCGR_ON),
    .irq = IRQ_LPUART8,
    .irq_handler = uart_interrupt_handler,
    .rx_pin = {
        .pin = RX_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT,
        .select_val = 1
    }
};

#else
#error "UART port not available!"
#endif

#ifdef SERIAL1_PORT

#if SERIAL1_PORT == SERIAL_PORT
#error Conflicting use of UART peripherals!
#endif

static uint16_t tx1_fifo_size;
DMAMEM static stream_tx_buffer_t tx1buffer;
DMAMEM static stream_rx_buffer_t rx1buffer;
static enqueue_realtime_command_ptr enqueue_realtime_command1 = protocol_enqueue_realtime_command;

static const io_stream_t *serial1Init (uint32_t baud_rate);
static void uart1_interrupt_handler (void);

#define UART1 uart1_hardware

#if SERIAL1_PORT == 1

#define RX1_PIN 25
#define TX1_PIN 24

static const uart_hardware_t uart1_hardware =
{
    .port = &IMXRT_LPUART1,
    .ccm_register = &CCM_CCGR5,
    .ccm_value = CCM_CCGR5_LPUART1(CCM_CCGR_ON),
    .irq = IRQ_LPUART1,
    .irq_handler = uart1_interrupt_handler,
    .rx_pin = {
        .pin = RX1_PIN,
        .mux_val = 2,
        .select_reg = NULL,
        .select_val = 0
    },
    .tx_pin = {
        .pin = TX1_PIN,
        .mux_val = 2,
        .select_reg = NULL,
        .select_val = 0
    }
};

#elif SERIAL1_PORT == 5

#define RX1_PIN 34
#define TX1_PIN 35

static const uart_hardware_t uart1_hardware =
{
    .port = &IMXRT_LPUART5,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART5(CCM_CCGR_ON),
    .irq = IRQ_LPUART5,
    .irq_handler = uart1_interrupt_handler,
    .rx_pin = {
        .pin = RX1_PIN,
        .mux_val = 1,
        .select_reg = &IOMUXC_LPUART5_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX1_PIN,
        .mux_val = 1,
        .select_reg = &IOMUXC_LPUART5_TX_SELECT_INPUT,
        .select_val = 1
    }
};

#elif SERIAL1_PORT == 6

#define RX1_PIN 0
#define TX1_PIN 1

static const uart_hardware_t uart1_hardware =
{
    .port = &IMXRT_LPUART6,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART6(CCM_CCGR_ON),
    .irq = IRQ_LPUART6,
    .irq_handler = uart1_interrupt_handler,
    .rx_pin = {
        .pin = RX1_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART6_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX1_PIN,
        .mux_val = 2,
        .select_reg = NULL,
        .select_val = 0
    }
};

#elif SERIAL1_PORT == 8

#define RX1_PIN 21
#define TX1_PIN 20

static const uart_hardware_t uart1_hardware =
{
    .port = &IMXRT_LPUART8,
    .ccm_register = &CCM_CCGR6,
    .ccm_value = CCM_CCGR6_LPUART8(CCM_CCGR_ON),
    .irq = IRQ_LPUART8,
    .irq_handler = uart1_interrupt_handler,
    .rx_pin = {
        .pin = RX1_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_RX_SELECT_INPUT,
        .select_val = 1
    },
    .tx_pin = {
        .pin = TX1_PIN,
        .mux_val = 2,
        .select_reg = &IOMUXC_LPUART8_TX_SELECT_INPUT,
        .select_val = 1
    }
};

#endif

#endif // SERIAL1_PORT

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit
    }
#ifdef SERIAL1_PORT
  , {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial1Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .pin = TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "UART1"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .pin = RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "UART1"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

#ifdef SERIAL1_PORT

    static const periph_pin_t tx1 = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .pin = TX1_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "UART2"
    };

    static const periph_pin_t rx1 = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .pin = RX1_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "UART2"
    };

    hal.periph_port.register_pin(&rx1);
    hal.periph_port.register_pin(&tx1);

#endif

    stream_register_streams(&streams);
}

static bool setBaudRate (const uart_hardware_t *uart, uint32_t baud_rate)
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

    uart->port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv) | (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);

    return true;
}

static uint32_t uartConfig (const uart_hardware_t *uart, uint32_t baud_rate)
{
    uint32_t tx_fifo_size;

    *uart->ccm_register |= uart->ccm_value;

    *(portControlRegister(uart->rx_pin.pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(uart->rx_pin.pin)) = uart->rx_pin.mux_val;
    if (uart->rx_pin.select_reg)
        *(uart->rx_pin.select_reg) = uart->rx_pin.select_val;

    *(portControlRegister(uart->tx_pin.pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(uart->tx_pin.pin)) = uart->tx_pin.mux_val;
    if (uart->tx_pin.select_reg)
        *(uart->tx_pin.select_reg) = uart->tx_pin.select_val;

    setBaudRate(uart, baud_rate);

    uart->port->PINCFG = 0;

    // Enable the transmitter, receiver and enable receiver interrupt
    NVIC_DISABLE_IRQ(uart->irq);
    attachInterruptVector(uart->irq, uart->irq_handler);
    NVIC_SET_PRIORITY(uart->irq, 0);
    NVIC_ENABLE_IRQ(uart->irq);

    tx_fifo_size = (uart->port->FIFO >> 4) & 0x7;
    tx_fifo_size = tx_fifo_size ? (2 << tx_fifo_size) : 1;

    uint8_t tx_water = (tx_fifo_size < 16) ? tx_fifo_size >> 1 : 7;
    uint16_t rx_fifo_size = (((uart->port->FIFO >> 0) & 0x7) << 2);
    uint8_t rx_water = (rx_fifo_size < 16) ? rx_fifo_size >> 1 : 7;

    uart->port->WATER = LPUART_WATER_RXWATER(rx_water) | LPUART_WATER_TXWATER(tx_water);
    uart->port->FIFO |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;
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
    uart->port->CTRL = ctrl;

    // Bit 3 10 bit - Will assume that begin already cleared it.
    // process some other bits which change other registers.
    if (format & 0x08)
        uart->port->BAUD |= LPUART_BAUD_M10;

    // Bit 4 RXINVERT
    uint32_t c = uart->port->STAT & ~LPUART_STAT_RXINV;
    if (format & 0x10)
        c |= LPUART_STAT_RXINV;      // rx invert
    uart->port->STAT = c;

    // bit 8 can turn on 2 stop bit mode
    if (format & 0x100)
        uart->port->BAUD |= LPUART_BAUD_SBNS;

    return tx_fifo_size;
}

// *******

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
    return setBaudRate(&UART, baud_rate);
}

static bool serialDisable (bool disable)
{
    if(disable)
        UART.port->CTRL &= ~LPUART_CTRL_RIE;
    else
        UART.port->CTRL |= LPUART_CTRL_RIE;

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static const io_stream_t *serialInit (uint32_t baud_rate)
{
    PROGMEM static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n = serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    memset(&rxbuffer, 0, sizeof(stream_rx_buffer_t));
    memset(&txbuffer, 0, sizeof(stream_tx_buffer_t));

    tx_fifo_size = uartConfig(&UART, baud_rate);

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

#ifdef SERIAL1_PORT

//
// serial1GetC - returns -1 if no data available
//
static int16_t serial1GetC (void)
{
    uint_fast16_t tail = rx1buffer.tail;         // Get buffer pointer

    if(tail == rx1buffer.head)
        return -1; // no data available

    int16_t data = rx1buffer.data[tail];         // Get next character, increment tmp pointer
    rx1buffer.tail = BUFNEXT(tail, rx1buffer);    // and update pointer

    return data;
}

static void serial1TxFlush (void)
{
    tx1buffer.tail = tx1buffer.head;
}

static uint16_t serial1RxCount (void)
{
    uint_fast16_t head = rx1buffer.head, tail = rx1buffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serial1RxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serial1RxCount();
}

static void serial1RxFlush (void)
{
    rx1buffer.tail = rx1buffer.head;
    rx1buffer.overflow = false;
}

static void serial1RxCancel (void)
{
    serial1RxFlush();
    rx1buffer.data[rx1buffer.head] = ASCII_CAN;
    rx1buffer.head = BUFNEXT(rx1buffer.head, rx1buffer);
}

static bool serial1PutC (const char c)
{
    if(tx1buffer.head == tx1buffer.tail && ((UART1.port->WATER >> 8) & 0x7) < tx1_fifo_size) {
        UART1.port->DATA  = c;
        return true;
    }

    uint_fast16_t next_head = BUFNEXT(tx1buffer.head, tx1buffer);   // Get next head pointer

    while(tx1buffer.tail == next_head) {             // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    tx1buffer.data[tx1buffer.head] = c;               // Add data to buffer
    tx1buffer.head = next_head;                      // and update head pointer

    __disable_irq();
    UART1.port->CTRL |= LPUART_CTRL_TIE; // (may need to handle this issue)BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT); // Enable TX interrupts
    __enable_irq();

    return true;
}

static void serial1WriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serial1PutC(c);
}

static void serial1Write(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial1PutC(*ptr++);
}

static bool serial1SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rx1buffer, suspend);
}

static uint16_t serial1TxCount(void)
{
    uint_fast16_t head = tx1buffer.head, tail = tx1buffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART1.port->WATER >> 8) & 0x7) + ((UART1.port->STAT & LPUART_STAT_TC) ? 0 : 1);
}

static bool serial1SetBaudRate (uint32_t baud_rate)
{
    return setBaudRate(&UART1, baud_rate);
}

static bool serial1Disable (bool disable)
{
    if(disable)
        UART1.port->CTRL &= ~LPUART_CTRL_RIE;
    else
        UART1.port->CTRL |= LPUART_CTRL_RIE;

    return true;
}

static bool serial1EnqueueRtCommand (char c)
{
    return enqueue_realtime_command1(c);
}

static enqueue_realtime_command_ptr serial1SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command1;

    if(handler)
        enqueue_realtime_command1 = handler;

    return prev;
}

static const io_stream_t *serial1Init (uint32_t baud_rate)
{
    PROGMEM static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .is_connected = stream_connected,
        .read = serial1GetC,
        .write = serial1WriteS,
        .write_n = serial1Write,
        .write_char = serial1PutC,
        .enqueue_rt_command = serial1EnqueueRtCommand,
        .get_rx_buffer_free = serial1RxFree,
        .get_rx_buffer_count = serial1RxCount,
        .get_tx_buffer_count = serial1TxCount,
        .reset_write_buffer = serial1TxFlush,
        .reset_read_buffer = serial1RxFlush,
        .cancel_read_buffer = serial1RxCancel,
        .suspend_read = serial1SuspendInput,
        .disable_rx = serial1Disable,
        .set_baud_rate = serial1SetBaudRate,
        .set_enqueue_rt_handler = serial1SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

    memset(&rx1buffer, 0, sizeof(stream_rx_buffer_t));
    memset(&tx1buffer, 0, sizeof(stream_tx_buffer_t));

    tx1_fifo_size = uartConfig(&UART1, baud_rate);

    return &stream;
}

static void uart1_interrupt_handler (void)
{
    uint32_t ctrl = UART1.port->CTRL;

    if((ctrl & LPUART_CTRL_TIE) && (UART1.port->STAT & LPUART_STAT_TDRE))
    {
        uint_fast16_t tail = tx1buffer.tail;                // Get buffer pointer

        do {
            if(tx1buffer.head != tail) {
                UART1.port->DATA = tx1buffer.data[tail];    // Put character in TXT register
                tail = BUFNEXT(tail, tx1buffer);            // and update tmp tail pointer
            } else
                break;
        } while(((UART1.port->WATER >> 8) & 0x7) < tx1_fifo_size);

        if((tx1buffer.tail = tail) == tx1buffer.head)  // Disable TX interrups
            UART1.port->CTRL &= ~LPUART_CTRL_TIE;
    }

    if ((ctrl & LPUART_CTRL_TCIE) && (UART1.port->STAT & LPUART_STAT_TC))
        UART1.port->CTRL &= ~LPUART_CTRL_TCIE;

    if(UART1.port->STAT & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) {

        while((UART1.port->WATER >> 24) & 0x7) {
            uint32_t data = UART1.port->DATA & 0xFF;                            // Read input (use only 8 bits of data)
            if(!enqueue_realtime_command1((char)data)) {
                uint_fast16_t next_head = BUFNEXT(rx1buffer.head, rx1buffer);   // Get next head pointer
                if(next_head == rx1buffer.tail)                                 // If buffer full
                    rx1buffer.overflow = true;                                  // flag overflow
                else {
                    rx1buffer.data[rx1buffer.head] = (char)data;                // Add data to buffer
                    rx1buffer.head = next_head;                                 // and update pointer
                }
            }
        }

        if (UART1.port->STAT & LPUART_STAT_IDLE)
            UART1.port->STAT |= LPUART_STAT_IDLE; // writing a 1 to idle should clear it.
    }
}

#endif // SERIAL1_PORT
