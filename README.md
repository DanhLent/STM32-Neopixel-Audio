# STM32 WS2812B Music Visualizer & Light Show

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

### Introduction
This repository contains the source code for an academic project demonstrating a real-time, audio-reactive light show. The system utilizes an STM32F407VET6 microcontroller to drive a WS2812B (Neopixel) addressable LED strip, creating visual effects including a vibrant rainbow wave and a dynamic music visualizer. A user-friendly interface is implemented with an LCD and a rotary encoder for seamless effect selection.

---

### Table of Contents
- [Demo](#demo)
- [Key Features](#key-features)
- [Hardware & Software Requirements](#hardware--software-requirements)
- [Pinout Configuration](#pinout-configuration)
- [Peripheral Configuration](#peripheral-configuration)
- [Installation](#installation)
- [Credits](#credits)

---

### Demo
*https://www.youtube.com/watch?v=HTsP6H0X7IE*

---

### Key Features
-   **Real-time Music Visualization:** Analyzes audio intensity from a microphone to generate synchronized lighting effects.
-   **Dynamic Lighting Effects:** Includes a smooth, pre-programmed "Rainbow Wave" effect.
-   **Interactive UI:** A simple interface with an ILI9341 LCD and a Rotary Encoder allows for easy navigation and mode switching.

---

### Hardware & Software Requirements

#### Hardware
| Component | Model | Role |
| :--- | :--- | :--- |
| Microcontroller | STM32F407VET6 | Central processing, controls all peripherals |
| LED Strip | WS2812B (Neopixel) | Displays visual effects |
| Audio Module | MAX9814 Microphone | Captures ambient sound |
| Display | ILI9341 TFT LCD | Displays the user interface |
| Input | Rotary Encoder | User input for menu navigation |
| Power Supply | 5V, 3A | Powers the MCU and LED strip |

#### Software
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- An ST-Link programmer/debugger

---

### Pinout Configuration
<img width="864" height="799" alt="Screenshot 2025-07-20 130626" src="https://github.com/user-attachments/assets/62185ba7-e927-4666-a1b7-b867f5ce4381" />



---

### Peripheral Configuration
Key peripherals were configured for optimal performance. Below are screenshots of the main settings in STM32CubeIDE.

System Clock Configuration

The system is configured to run at a maximum frequency of 72MHz, derived from an 8MHz external crystal oscillator (HSE). This ensures the highest processing performance for effect algorithms and audio analysis.
<img width="1425" height="693" alt="image" src="https://github.com/user-attachments/assets/e228076f-3079-4436-a939-41058ce26e32" />


TIM1 for WS2812B (PWM & DMA)

Timer 1 is configured to generate an 800kHz PWM signal, precisely matching the WS2812B protocol's requirements. Calculation: 72,000,000 Hz / (Period + 1) = 800,000 Hz. DMA is set to Memory-to-Peripheral mode to automatically transfer data, completely offloading the CPU.

<img width="1131" height="252" alt="image" src="https://github.com/user-attachments/assets/c61780bf-057d-4709-af8b-51c939931c8b" />

<img width="1120" height="243" alt="image" src="https://github.com/user-attachments/assets/b45c01cc-f85c-4ccf-bedf-8c3b9eafc742" />


TIM2 & ADC1 for Audio Sampling

Timer 2 acts as a trigger for ADC1 to ensure a consistent audio sampling rate. ADC1 and DMA are configured in Circular Mode to continuously sample audio data into a buffer without CPU intervention.

<img width="1216" height="358" alt="image" src="https://github.com/user-attachments/assets/567fbaf5-463d-40ee-8ce6-482e88154295" />

<img width="1404" height="633" alt="image" src="https://github.com/user-attachments/assets/6c7e3956-557b-44f4-9aba-5c3a14accbf1" />



---

### Installation
<details>
  <summary>Click to view installation guide</summary>
  
  1. Clone the repository to your local machine:
     ```bash
     git clone https://github.com/DanhLent/STM32-Neopixel-Audio.git
     ```
  2. Open the project in STM32CubeIDE, connect the hardware according to the pinout diagram, then build and flash to the board.
</details>

---

### Credits
-   The ILI9341 LCD driver is based on the **[STM32-ILI9341-FSMC-Library](https://github.com/taburyak/STM32-ILI9341-320x240-FSMC-Library)**.
-   The rainbow effect algorithm is inspired by the work of **[Adriano Tiger's Neopixel Effect Generator](https://adrianotiger.github.io/Neopixel-Effect-Generator/)**.
