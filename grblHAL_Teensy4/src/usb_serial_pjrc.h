/*

  usb_serial.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "usb_serial.h"

#include "grbl/hal.h"

#define usb_serial_input() usb_serial_available()

const io_stream_t *usb_serialInit(void);
void usb_execute_realtime (void);

#endif
