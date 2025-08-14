# TMC5x Driver Library

A lightweight driver library for **TMC5160** and **TMC5130** stepper motor driver chips, covering most common use cases.

## Features

- Supports TMC5160 and TMC5130.
- Covers most motion control scenarios.
- Platform-independent design.

## Porting Guide

To port this library to your platform:

1. Implement the predefined interfaces in the `prot` file.
2. Fill in the `tmc51x0_info` structure in the `tmc51x0_lib_port.h` file according to your platform.
3. Supports using the TMC51x0 **Ramp Generator** to drive the motor directly.
    If you prefer **step/dir** signals, you need to implement motion signal control separately.

## API

The API is designed to be self-explanatory.
 Check the source code for detailed usage examples.