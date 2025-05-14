## Audio Recorder

## Description

This project is an STM32-based audio recorder that captures sound from a microphone and stores it on an SD card. 

## Features

* Audio recording from an analog microphone.
* Storage of audio data on an SD card.
* Uses the STM32 HAL library for hardware abstraction.
* Implements a circular buffer with DMA for efficient data handling.
* Uses the FatFs library for SD card file system management.
* Provides clear and concise code with detailed comments.

## Hardware

* STM32F4F407 microcontroller 
* Microphone (analog)
* SD card and SD card reader
* STM32 development board
* Wires for connections

## Software

* STM32 HAL library
* FatFs library
* C programming language
* STM32CubeIDE or other STM32 development environment

## How it Works

1.  The microphone captures audio signals.
2.  The STM32's ADC (Analog-to-Digital Converter) converts the analog audio signal into digital data.
3.  DMA (Direct Memory Access) transfers the digital audio data from the ADC to a buffer in memory.
4.  The STM32 writes the audio data from the buffer to an SD card as a WAV file.
5.  The process repeats, continuously recording audio.

## Code Structure

* `main.c`: Contains the main program logic, initialization, and main loop.
* `SystemClock_Config()`: Configures the STM32 system clock.
* `MX_GPIO_Init()`: Initializes the GPIO pins for the microphone and SD card.
* `MX_DMA_Init()`: Initializes the DMA for transferring data from the ADC.
* `MX_ADC1_Init()`: Initializes the ADC to read audio data.
* `MX_TIM2_Init()`: Initializes Timer 2 to trigger ADC conversions at a specific sample rate.
* `HAL_ADC_ConvHalfCpltCallback()`/`HAL_ADC_ConvCpltCallback()`:  Handle ADC data transfer interrupts.
* `convert_to_pcm16()`: Converts the 12-bit ADC value to a 16-bit PCM value.
* `SD_Init()`: Initializes the SD card.
* `SD_Write()`: Writes audio data to the SD card.
* `SD_Error_Handler()`: Handles SD card errors.
* `Error_Handler()`: Handles general errors.

## Wiring
Connect the hardware as follows
* Microphone to PA0
* SD Card to SPI pins 
* SD Card CS to PB12


## To Do

* Add a WAV file header to the recorded data.
* Implement error handling for file operations.
* Add support for different sample rates.
* Optimize code for lower power consumption.
* Add user interface (e.g., LEDs, buttons) for controlling the recording.

## Contributions

Contributions are welcome! Feel free to submit pull requests or open issues to suggest improvements or report bugs.


