# STM32 WS2812B Music Visualizer & Light Show

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

### Introduction
This repository contains the source code for a university project demonstrating a real-time, audio-reactive light show. The system utilizes an STM32F407VET6 microcontroller to drive a WS2812B (Neopixel) addressable LED strip, creating complex visual effects including a vibrant rainbow wave and a dynamic music visualizer. A user-friendly interface is implemented with an LCD and a rotary encoder for seamless effect selection.

---

### Table of Contents
- [Demo](#demo)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Hardware & Software Requirements](#hardware--software-requirements)
- [Pinout Configuration](#pinout-configuration)
- [Peripheral Configuration](#peripheral-configuration)
- [Installation](#installation)
- [Credits](#credits)

---

### Demo
*(Đây là phần để bạn chèn video minh chứng. Hãy xem hướng dẫn ở cuối câu trả lời này nhé.)*

---

### Key Features
-   **Real-time Music Visualization:** Analyzes audio intensity from a microphone to generate synchronized lighting effects.
-   **Dynamic Lighting Effects:** Includes a smooth, pre-programmed "Rainbow Wave" effect.
-   **Interactive UI:** A simple interface with an ILI9341 LCD and a Rotary Encoder allows for easy navigation and mode switching.

---

### System Architecture
The system is designed with a modular architecture, centered around the STM32 microcontroller which handles all processing, peripheral control, and logic.
*(Chèn ảnh sơ đồ khối từ file báo cáo của bạn vào đây)*
![Block Diagram](link_den_so_do_khoi.png)

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
- Git

---

### Pinout Configuration
This image shows the pin connections configured in STM32CubeIDE.
*(Đây là chỗ để bạn chèn ảnh `image_2914f8.png` mà bạn đã gửi)*
![Pinout Diagram](link_den_anh_pinout.png)

---

### Peripheral Configuration
Key peripherals were configured for optimal performance. Below are screenshots of the main settings in STM32CubeIDE.

#### System Clock Configuration
*(Chèn ảnh cấu hình Clock của bạn vào đây)*
![Clock Configuration](link_den_anh_cau_hinh_clock.png)

#### TIM1 for WS2812B (PWM & DMA)
*(Chèn ảnh cấu hình TIM1 của bạn vào đây)*
![TIM1 Configuration](link_den_anh_cau_hinh_tim1.png)

#### TIM2 & ADC1 for Audio Sampling
*(Chèn ảnh cấu hình TIM2 và ADC1 của bạn vào đây)*
![ADC Configuration](link_den_anh_cau_hinh_adc1.png)

---

### Installation
1.  Clone the repository to your local machine:
    ```bash
    git clone [https://github.com/DanhLent/STM32-Neopixel-Audio.git](https://github.com/DanhLent/STM32-Neopixel-Audio.git)
    ```
2.  Open STM32CubeIDE and import the project (`File > Import > General > Existing Projects into Workspace`).
3.  Connect the hardware according to the pinout diagram.
4.  Build the project and flash it to the STM32 board.

---

### Credits
-   The ILI9341 LCD driver is based on the **[STM32-ILI9341-FSMC-Library](https://github.com/taburyak/STM32-ILI9341-320x240-FSMC-Library)**.
-   The rainbow effect algorithm is inspired by the work of **[Adriano Tiger's Neopixel Effect Generator](https://adrianotiger.github.io/Neopixel-Effect-Generator/)**.
