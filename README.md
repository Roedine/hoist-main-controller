# Hoist Main Controller

STM32-based main controller developed for an automated hoist system as part of a university engineering design project.

## My Contribution

I was responsible for the **main controller firmware**, which manages communication, operating states and motor control for the hoist system.

## Key Features

* Embedded C on an STM32F411 microcontroller
* FreeRTOS task management
* Finite State Machine (FSM)
* nRF24 wireless communication via SPI
* PWM motor control
* Three-phase sine-wave generation
* Variable frequency (V/f) motor control
* Motor speed and direction control
* Emergency-stop functionality
* Communication-loss failsafe
* UART status communication

## Firmware Structure

The controller uses separate FreeRTOS tasks for:

* **FSM** – manages the operating states of the hoist
* **PWM Control** – handles motor frequency and three-phase PWM generation
* **Wireless Communication** – receives commands from the remote controller

The controller also includes safety logic that stops the motors when an emergency stop is activated or communication with the remote controller is lost.

## Technologies

`C` `STM32F411` `STM32CubeIDE` `FreeRTOS` `SPI` `UART` `nRF24L01` `PWM`

## Project Context

This was a team engineering project. This repository contains the **main controller firmware**, which was my primary responsibility.

## Author

**Roedine van der Merwe**
B.Eng Computer & Electronic Engineering
