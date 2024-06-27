<!--
    Copyright 2024, UNSW

    SPDX-License-Identifier: BSD-2-Clause
-->

# Network echo server

## Building
```sh
make BUILD_DIR=<path/to/build> MICROKIT_SDK=<path/to/sdk> MICROKIT_CONFIG=(benchmark/release/debug)
```

## Benchmarking

In order to run the benchmarks, set `MICROKIT_CONFIG=benchmark`. The system has
been designed to interact with [ipbench](https://sourceforge.net/projects/ipbench/)
to take measurements.

Checks to make before benchmarking:
* Turn off all debug prints.
* Run with LWIP asserts turned off as well (`LWIP_NOASSERT`).
* Make sure compiler optimisations are enabled.
