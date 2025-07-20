# STM32 Music Visualizer & Light Show

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Made with: C/C++](https://img.shields.io/badge/Made%20with-C%2F C%2B%2B-blue)](https://en.cppreference.com/)
[![Hardware: STM32](https://img.shields.io/badge/Hardware-STM32-orange)](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)

A real-time audio-reactive LED strip controller powered by an STM32F4 microcontroller. This project visualizes music through various dynamic light effects on a WS2812B (Neopixel) LED strip.

---

## 📋 Table of Contents

- [Demo](#-demo)
- [Key Features](#-key-features)
- [Tech Stack](#-tech-stack--components)
- [System Architecture](#-system-architecture)
- [Technical Highlights](#-technical-highlights)
- [Installation & Usage](#-installation--usage)
- [Future Improvements](#-future-improvements)
- [Credits](#-credits)

---

## 📸 Demo

*(This is the most critical section for your project. A GIF is highly recommended.)*

![Demo GIF](link_to_your_gif.gif)

---

## ✨ Key Features

-   **Real-time Music Visualization:** Analyzes audio intensity from a microphone to generate synchronized lighting effects.
-   **Multiple Effects:** Includes a smooth "Rainbow Wave" effect in addition to the music mode.
-   **User Interface:** A simple UI with an LCD and a Rotary Encoder allows for easy navigation and effect selection.

---

## 🛠️ Tech Stack & Components

#### Hardware
-   **Microcontroller:** STM32F407VET6
-   **LEDs:** WS2812B Addressable RGB LED Strip
-   **Audio Input:** MAX9814 Microphone Module
-   **Display:** ILI9
-   **Input:** Rotary Encoder with push button

#### Software & Tools
-   **Language:** C/C++
-   **IDE:** STM32CubeIDE
-   **Libraries:** STM32 HAL, ARM CMSIS

---

## 🏗️ System Architecture

*(You can export the block diagram image from your project report and insert it here.)*

![Block Diagram](link_to_your_diagram.png)

---

## 🔧 Technical Highlights

This section details the key engineering decisions made to optimize the system's performance.

-   **Non-Blocking LED Control:** Utilizes a combination of **Timer PWM and DMA** to generate the precise 800kHz signal for the WS2812B LEDs. This approach completely offloads the CPU, allowing it to focus on complex effect calculations.
-   **Efficient Audio Sampling:** Employs an **ADC with DMA (in Circular Mode)** to continuously sample the audio signal without CPU intervention, ensuring smooth and responsive visualizations.
-   **High-Speed Display Interface:** Leverages the **FSMC** (Flexible Static Memory Controller) peripheral for a parallel interface with the LCD, achieving significantly higher refresh rates compared to SPI.
-   **Robust Software Architecture:** Built upon a **State Machine** design pattern for clear, maintainable, and easily extendable management of different operational modes.

---

## ⚙️ Installation & Usage

### Prerequisites
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- An ST-Link programmer/debugger

### Steps
1.  Clone the repository to your local machine:
    ```bash
    git clone [https://github.com/DanhLent/STM32-Neopixel-Audio.git](https://github.com/DanhLent/STM32-Neopixel-Audio.git)
    ```
2.  Open STM32CubeIDE and import the project (`File > Import > General > Existing Projects into Workspace`).
3.  Connect the hardware according to the schematics (you can add a picture of the wiring diagram here).
4.  Build the project and flash it to the STM32 board.

---

## 🚀 Future Improvements

-   Implement **FFT (Fast Fourier Transform)** for frequency-based audio analysis (e.g., reacting differently to bass, mids, and treble).
-   Design a custom **PCB** for a more compact and reliable product.
-   Add **wireless connectivity** (Bluetooth or Wi-Fi) to control the effects from a mobile application.

---

## 🙏 Credits

-   The LCD driver is based on the excellent **[STM32-ILI9341-FSMC Library by @taburyak](https://github.com/taburyak/STM32-ILI9341-FSMC-Library)**.
