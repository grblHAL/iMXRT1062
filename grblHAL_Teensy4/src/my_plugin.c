/*

  my_plugin.c - plugin template for setting auxillary output on feed hold

  Part of grblHAL

  Public domain.

*/

#include <string.h>

#include "driver.h"
#include "grbl/protocol.h"

static uint8_t port;

static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;

static void onStateChanged (sys_state_t state)
{
    static sys_state_t last_state = STATE_IDLE;

    if(state != last_state) {
        last_state = state;
        hal.port.digital_out(port, state == STATE_HOLD);
    }

    if(on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);  // Call previous function in the chain.

    if(!newopt)                 // Add info about us to the $I report.
        hal.stream.write("[PLUGIN:MY PLUGIN Template 2]" ASCII_EOL);
}

static void output_warning (uint_fast16_t state)
{
    report_message("An output port is required for my_plugin!", Message_Warning);
}

// Tell the user about which port is used for the output
static void output_port (uint_fast16_t state)
{
    char msg[30];

    strcpy(msg, "My plugin port: ");
    strcat(msg, uitoa(port));

    report_message(msg, Message_Info);
}

void xmy_plugin_init()
{
    if(hal.port.num_digital_out == 0)                   // This plugin requires one digital output port,
        protocol_enqueue_rt_command(output_warning);    // complain if not available.

    else {
        port = hal.port.num_digital_out - 1;            // Claim the
        hal.port.num_digital_out--;                     // last free port.

        on_state_change = grbl.on_state_change;         // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = onStateChanged;          // function pointer and adding ours to the chain.

        on_report_options = grbl.on_report_options;     // Add our plugin to to the options report chain
        grbl.on_report_options = onReportOptions;       // to tell the user we are active.

        protocol_enqueue_rt_command(output_port);       // Tell the user about which port is used for the output after startup is complete.
    }
}
