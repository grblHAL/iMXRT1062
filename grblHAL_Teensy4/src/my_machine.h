/*
  my_machine.h - configuration for IMXRT1062 processor (on Teensy 4.x board)

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// BOARD_T40X101 and BOARD_T41U5XBB by Phil Barrett: https://github.com/phil-barrett/grblHAL-teensy-4.x

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_T40X101
//#define BOARD_T41U5XBB
//#define BOARD_T41U5XBB_SS // For a modified T41U5XBB board, allows spindle sync to be enabled.
//#define BOARD_T41BB5X_PRO
//#define BOARD_CNC_BOOSTERPACK
//#define BOARD_GRBLHAL2000
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!
//#define BAUD_RATE 230400
// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

/*
              Plugin: | ETHERNET¹ | SDCARD¹ | KEYPAD | EEPROM | N_AXIS |
----------------------|-----------|---------|--------|--------|--------|
BOARD_T40X101         | no        | no      | yes    | yes³   | max 4  |
BOARD_T41U5XBB        | yes       | yes     | yes    | yes³   | max 5  |
BOARD_T41BB5X_PRO     | yes       | yes     | yes    | yes    | max 5  |
BOARD_CNC_BOOSTERPACK | yes²      | yes     | yes    | yes    | max 3  |
BOARD_GRBLHAL2000     | yes       | yes     | yes    | yes    | max 5  |

¹ Teensy 4.1 only
² External magjack.
³ EEPROM is optional and must be added to the board.

N_AXIS has a default value of 3, edit grbl\config.h to increase.

*/
#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC         2 // 1 for Arduino class library and 2 for PJRC C library. Comment out or set to 0 to use UART communication.
#endif
//#define USB_SERIAL_WAIT         1 // Wait for USB connection before starting grblHAL.
//#define BLUETOOTH_ENABLE        2 // Set to 2 for HC-05 module. Requires and claims one auxiliary input pin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE         SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE         SPINDLE_PWM0
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE2_ENABLE         SPINDLE_NONE
//#define SPINDLE_OFFSET          1 // Uncomment to enable settings for laser spindle XY-offset.
// **********************
//#define MODBUS_ENABLE           1 // Set to 1 for auto direction, 2 for direction signal on auxiliary output pin.
//#define WEBUI_ENABLE            3 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins.
//#define ETHERNET_ENABLE         1 // Ethernet streaming. Enables networking plugin.
//#define SDCARD_ENABLE           1 // Run gcode programs from SD card. Set to 2 to enable YModem upload.
//#define LITTLEFS_ENABLE         1 // Enable flash based storage, automatically enabled if WebUI is enabled. Set to 2 to mount as root.
//#define QEI_ENABLE              1 // Enable quadrature encoder interfaces. Max value is 1. Requires encoder plugin.
//#define MPG_ENABLE              1 // Enable MPG interface. Requires a serial stream and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial stream for input. If MPG_ENABLE is set > 0 the serial stream is shared with the MPG.
//#define DISPLAY_ENABLE          9 // Set to 9 for I2C display protocol, 17 for I2C LED protocol.
//#define MACROS_ENABLE           1 // Macros plugin. For macros that can be triggered by keypad plugin or auxiliary inputs.
                                    // Set to 1 for aux input triggers, 2 for keypad triggers or 3 for both.
//#define N_MACROS                1 // Number of macros to enable, max 8. If commented out default is 4 when triggered by aux inputs, 8 otherwise.
//#define PLASMA_ENABLE           1 // Plasma/THC plugin. To be completed.
//#define PPI_ENABLE              1 // Laser PPI plugin. To be completed.
//#define LASER_COOLANT_ENABLE    1 // Laser coolant plugin. To be completed.
//#define LASER_OVD_ENABLE        1 // Enable M-code for overdrive PWM output during spindle off in RPM controlled mode.
//#define LB_CLUSTERS_ENABLE      1 // LaserBurn cluster support.
//#define ODOMETER_ENABLE         1 // Odometer plugin.
//#define OPENPNP_ENABLE          1 // OpenPNP plugin. To be completed.
//#define FANS_ENABLE             1 // Enable fan control via M106/M107. Enables fans plugin.
//#define EEPROM_ENABLE          16 // I2C EEPROM/FRAM support. Set to 16 for 2K, 32 for 4K, 64 for 8K, 128 for 16K and 256 for 16K capacity.
//#define EEPROM_IS_FRAM          1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define SPINDLE_SYNC_ENABLE     1 // Enable spindle sync support (G33, G76). !! NOTE: Alpha quality - enable only for test or verification.
                                    // Currently only available for BOARD_T41BB5X_PRO and BOARD_T41U5XBB_SS.
//#define ESTOP_ENABLE            0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                                    // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.
//#define RGB_LED_ENABLE          2 // Set to 1 to enable strip length settings $536 and $537, set to 2 to also enable M150 LED strip control.
//#define PWM_SERVO_ENABLE        1 // Enable M280 PWM servo support, requires at least one PWM capable auxiliary output.
//#define BLTOUCH_ENABLE          1 // Enable M401/M402 BLTouch support. Requires and claims one auxiliary PWM servo output.
//#define EVENTOUT_ENABLE         1 // Enable binding events (triggers) to control auxiliary outputs.
//#define ESP_AT_ENABLE           1 // Enable support for Telnet communication via UART connected ESP32 running ESP-AT.
//#define FEED_OVERRIDE_ENABLE    1 // Enable M200 feed override control.
//#define HOMING_PULLOFF_ENABLE   1 // Enable per axis homing pulloff distance settings.

// IO expanders:
//
//#define MCP3221_ENABLE          1 // MCP3221 I2C ADC input, default address is 0x9A (MCP3221_ADDRESS).
//#define PCA9654E_ENABLE         1 // PCA9654E I2C digital I/O, default address is 0x40 (PCA9654E_ADDRESS).

// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define PROBE_ENABLE            0 // Default enabled, remove comment to disable probe input.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

// If the selected board map supports more than three motors ganging and/or auto-squaring
// of axes can be enabled here.
//#define X_GANGED             1
//#define X_AUTO_SQUARE        1
//#define Y_GANGED             1
//#define Y_AUTO_SQUARE        1
//#define Z_GANGED             1
//#define Z_AUTO_SQUARE        1
// For ganged axes the limit switch input (if available) can be configured to act as a max travel limit switch.
// NOTE: If board map already has max limit inputs defined this configuration will be ignored.
//#define X_GANGED_LIM_MAX     1
//#define Y_GANGED_LIM_MAX     1
//#define Z_GANGED_LIM_MAX     1
//

#if ETHERNET_ENABLE || WEBUI_ENABLE
//#define TELNET_ENABLE        1 // Telnet daemon - requires Ethernet streaming enabled.
//#define WEBSOCKET_ENABLE     1 // Websocket daemon - requires Ethernet streaming enabled.
//#define MDNS_ENABLE          1 // mDNS daemon.
//#define SSDP_ENABLE          1 // SSDP daemon - requires HTTP enabled.
//#define MQTT_ENABLE          1 // MQTT client API, only enable if needed by plugin code.
#if SDCARD_ENABLE || WEBUI_ENABLE
//#define FTP_ENABLE           1 // Ftp daemon - requires SD card enabled
//#define HTTP_ENABLE          1 // http daemon - requires SD card enabled.
//#define WEBDAV_ENABLE        1 // webdav protocol - requires http daemon and SD card enabled.
#endif
// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME        "grblHAL"
//#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"
//#define NETWORK_FTP_PORT        21
//#define NETWORK_TELNET_PORT     23
//#define NETWORK_HTTP_PORT       80
#if HTTP_ENABLE
//#define NETWORK_WEBSOCKET_PORT  81
#else
//#define NETWORK_WEBSOCKET_PORT  80
#endif
#endif

