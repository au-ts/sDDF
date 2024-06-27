<!--
    Copyright 2024, UNSW

    SPDX-License-Identifier: BSD-2-Clause
-->

# Serial example

This is an example to show multiple clients being used with a UART driver.

## Building

```sh
make MICROKIT_SDK=<path/to/sdk> MICROKIT_BOARD=<board> MICROKIT_CONFIG=<debug/release/benchmark>
```

Currently the options for `MICROKIT_BOARD` are:
* odroidc4
* imx8mm_evk
* maaxboard
* qemu_arm_virt

After building, the system image to load will be `build/loader.img`.

If you wish to simulate on the QEMU ARM virt platform, you can append `qemu` to your make command
after building for qemu_arm_virt.

## Configuration

In the serial example directory you will find the `include/serial_config/serial_config.h` file.
This file contains system configuration information that is dependent on your `.system` file, as
well as the following configuration options:

1. **SERIAL_TX_ONLY** - enable this if you only want to use the transmit functionality of the
serial subsystem. This stops the uart driver from enabling the receive functionality of the
device.
2. **SERIAL_WITH_COLOUR** - enable this if you want clients outputs to be different colours. This
mechanism works by appending a colour code before and after a clients string. Note that the
transmit virtualiser supports up to 256 colours. Also, the transmit virtualiser does not check
client output for colour sequences, so there is no gaurantee that clients will only output in
their own colour. Upon initialisation, the transmit virtualiser will print the name of each client
in the colour assigned to it.
3. **SERIAL_SWITCH_CHAR** and **SERIAL_TERMINATE_NUM** - these characters control the receive
virtualisers input switching mechanism. To switch the input stream to a different client, input
**SERIAL_SWITCH_CHAR** followed by up to 4 numeric characters corresponding to the new client
number, and terminate numeric input with **SERIAL_TERMINATE_NUM**. Upon success there will be no
output, while upon error the receive virtualiser will print a debug failure message. Client 0
receives input upon initialisation.
4. **UART_DEFAULT_BAUD** - this determines the baud rate that the uart driver will configure for
the device. Baud rate is always set explicitly instead of detected automatically.
5. **SERIAL_CONSOLE_BEGIN_STRING** - this string is printed by the transmit virtualiser upon
initialisation completion. This is to support input beginning in the interfacing serial server.

If the system file is changed, or the serial subsystem is included into another system, this config
file will need to be edited or re-created to reflect the new system. Be sure to check that the 
`*_init\_sys` functions correctly initialise each protection domains data structures.

## Interfacing with Other Systems
To include the serial subsystem into an existing system, the following steps must be taken:
* **.system File**
You must update your system file to include serial data and queue regions for each client and the
uart driver. You must also include the uart driver, transmit virtualiser, and optionally the
receive virtualiser protection domains. Finally you must include channels between your clients and
the virtualisers, as well as between the virtualisers and the uart driver.
* **`serial_config` File**
A new `serial_config` file must be created for your system, containing relevent details of the
system file including client names and queue sizes, as well as updated initialisation functions
for clients and virtualisers.
* **Makefile**
You must include directories for **SERIAL_COMPONENTS**, the **UART_DRIVER** and your
**SERIAL_CONFIG_INCLUDE**. You must also supply **SERIAL_NUM_CLIENTS**. You must add the uart
driver, transmit virtualiser and optionally the receive virtualiser to your image list. You must
add your serial include directory to your cflags, and finally you must include the uart driver
and serial_components make files. For each component you wish to have access to the serial
subsystem, you must link their printf object file with `libsddf_util.a` as opposed to
`libsddf_util_debug.a`. This will ensure printf invokes the serial _sddf_putchar.
* **Protection Domains**
Each protection domain that outputs to serial must include the serial queue library as well as
`serial_config.h`. They must also have the following declarations/definitions:

```
#define SERIAL_TX_CH 0

char *serial_tx_data;
serial_queue_t *serial_tx_queue;
serial_queue_handle_t serial_tx_queue_handle;
```

If they require serial input then equivalent declarations must exist for the receive serial
objects. Finally, during initialisation and prior to calling printf, they must initialise their 
serial queue(s) by calling `serial_cli_queue_init_sys` as well as `serial_putchar_init` which
allows them to also use `sddf_putchar_unbuffered`.

## Example
The serial server example system contains two clients which can both receive serial data as well
as transmit. By default, the example has SERIAL_WITH_COLOUR enabled so each client prints with a
different colour. Each client boots up and prints a hello world message when initialisation is
completed, and waits for input. When a character is received, each client will re-transmit the
character using `sddf_putchar_unbuffered` which flushes the character to the device immediately. Every
tenth character each client will print a string containing their name using `sddf_printf` which
calls the serial `_sddf_putchar`, flushing characters to the device only when a `\n` is
encountered.