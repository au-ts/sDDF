<!--
    Copyright 2024, UNSW

    SPDX-License-Identifier: BSD-2-Clause
-->

# I<sup>2</sup>C example

This example makes use of the NXP PN532 card-reader. This example was developed and tested using
the PN532 NFC RFID Module V3. Documentation on the card-reader itself can be found
on the [NXP website](https://www.nxp.com/docs/en/user-guide/141520.pdf).

This example is intended to run on an Odroid-C4 using the I2C connected via GPIO pins 3 and 5
for SDA and SCL respectively.

The client code for dealing with the card-reader is based on an Arduino PN532 library, it can
be found [here](https://github.com/elechouse/PN532/).

## Building

To build the image, run the following command:
```sh
make MICROKIT_SDK=/path/to/sdk
```

The final bootable image will be in `build/loader.img`.

## Running

Running the example will show the following output after the system has booted:
```
PN532|ERROR: read_response - length was less than zero
CLIENT|ERROR: Timed out waiting for a card
PN532|ERROR: read_response - length was less than zero
CLIENT|ERROR: Timed out waiting for a card
```

If you put a card in front of the card reader, you should see messages similar to
the following:
```
UID Length: 4 bytes
UID Value:  0x9c 0x1b 0xb4 0x1
UID Length: 4 bytes
UID Value:  0x9c 0x1b 0xb4 0x1
UID Length: 4 bytes
UID Value:  0x9c 0x1b 0xb4 0x1
```

where the card UID is being read and printed out.

Note that for MiFare Ultra Light cards or cards that have encryption, you may
see timeouts between each UID card read, this is expected.
