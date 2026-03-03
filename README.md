# imxrtnsy

Bare-metal USB proxy firmware for the Teensy 4.1 (NXP i.MX RT1062).

Plugs in between a USB device and a host PC. Captures the device's descriptors on the USB2 host port, then presents itself to the PC on USB1 as that same device, forwarding HID interrupt reports in both directions.

## Hardware

- Teensy 4.1
- USB host cable on the USB2 port (the one with pads on the bottom of the board)
- USB device cable to the PC on the normal USB1 port

The VBUS power switch is on GPIO_EMC_40 (GPIO8 bit 26).

## Building

Requires the ARM GCC toolchain. The Makefile expects it at the PlatformIO default path — edit `TOOLCHAIN` in the Makefile if yours is elsewhere.

```
make -j4
```

To enable UART debug output on Serial1 (pin 1 TX, 115200 baud):

```
make UART=1 -j4
```

Flash with the Teensy Loader or `teensy_loader_cli`.

## What it does

1. Initializes the USB2 EHCI host controller and waits for a device
2. Resets the port and runs full enumeration — device, config, HID report, and string descriptors
3. Sends SET_CONFIGURATION, SET_IDLE, and SET_PROTOCOL to the device
4. Starts the USB1 device controller, replaying the captured descriptors to the PC
5. Enters a proxy loop: polls interrupt IN endpoints on the host side and forwards reports to the PC via the device side

Supports composite devices with multiple HID interfaces (tested with a Razer Basilisk V3).

## Status

Work in progress. Device enumeration and descriptor replay are working. Interrupt report forwarding is under active development.

## Files

- `core/` — startup code, linker script, boot data
- `src/main.c` — main proxy flow and LED state machine
- `src/usb_host.c` — EHCI USB2 host driver (control + interrupt transfers)
- `src/usb_device.c` — USB1 device controller (descriptor replay + report forwarding)
- `src/desc_capture.c` — full device enumeration and descriptor capture
- `src/uart.c` — LPUART6 debug output
- `include/imxrt.h` — i.MX RT1062 register definitions
