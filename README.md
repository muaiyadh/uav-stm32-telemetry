# STM32F407 + nRF24L01+ Telemetry Firmware

> **Status:** Archived research snapshot.
> I no longer have the hardware. Code is published *as-is* for reference and verification.

This repository contains the exact CubeIDE project I used for UAV radio-link experiments in 2023.
It runs on an STM32F407G-DISC1 board wired to an nRF24L01+ module and transmits 32-byte payloads at 2 Mbps on channel 2476 MHz, using basic error-checking methods.

Notably, the firmware toggles the STM32’s hardware CRC peripheral to benefit from fast hardware CRC calculation when needed, but omits CRC transmission on certain command packets for speed.

## Building

* STM32CubeIDE v1.12.0
* STM32 HAL F4 v1.27.1
