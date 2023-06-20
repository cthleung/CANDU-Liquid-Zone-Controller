# Embedded Programming Project - CANDU Liquid Zone Controller

This project is a collaboration between two members to design and implement a PID control system for the Liquid Zone Control (LZC) of a CANDU Nuclear Reactor. The control system was tested using a hardware-in-loop (HITL) CANDU plant simulator loaded on an Arduino Nano. The PID controller reads a pulse-width-modulated (PWM) signal from the Arduino, processes it via a TI MSP-430 microcontroller, and feeds the output back into the Arduino for feedback control. The results are displayed in MATLAB software.

## Table of Contents
- [Introduction](#introduction)
- [Project Description](#project-description)
- [Role and Responsibilities](#role-and-responsibilities)
- [Code Description](#code-description)
- [Results](#results)
- [How to Run](#how-to-run)

## Introduction
The project is a 5-week task aiming to implement a PID control system for a simulated CANDU nuclear reactor. The control system is designed using digital and analog components available in an electronics laboratory. It is a hands-on project intended to familiarize the team with embedded programming concepts and C programming.

## Project Description
The project uses an Arduino Nano to simulate the CANDU Nuclear Reactor. The control system reads a PWM signal from the Arduino, representing the liquid level of the LZC. This signal is digitized, sampled, and operated on by the PID control system within a TI MSP-430 microcontroller. The output of the MSP-430 is used as feedback control for the Arduino, and the sampled input of the MSP-430 is visually displayed in MATLAB software.

## Role and Responsibilities
As a part of the team, my responsibilities included:
- Developing C code to apply control theory (PID control) to vary the Liquid Zone (neutron absorber) in response to simulated stochastic neutron flux.
- Optimizing/tuning PID terms to minimize over/undershoot and steady state error.
- Streamlining C code for deployment on MSP-430 microcontroller by ensuring that stack allocation was error-free and resolving raised CPU flags.

## Code Description
The C code we developed initializes the UART and ADC modules, performs sampling, reads PWM, and applies the PID functions. It displays the power level approaching the desired set point, albeit with occasional fluctuations. The code leverages functionalities from the `io430.h` library.

## Results
Our implementation was largely successful in achieving the desired objectives of implementing PID control to the input signal. While the power level demonstrated some fluctuations, it generally approached the set point as expected. It shows that the design objectives have been largely met, albeit with room for further optimization, particularly with regards to tuning the PID parameters.

## How to Run
The project uses an Arduino Nano and a Texas Instruments MSP-430 microcontroller. The user should ensure they have access to these hardware components and the MATLAB software. The C code should be loaded onto the MSP-430 microcontroller. Upon running the control system, the user can observe the results visually in MATLAB software.
