# STM32 Music Visualizer & Light Show

This is a university project for the Microprocessors & Microcontrollers course. It utilizes an STM32F407VET6 microcontroller to drive a WS2812B (Neopixel) LED strip, creating various lighting effects, including a real-time music visualizer.

---

## 📸 Demo

**(MOST IMPORTANT)**
*A GIF or YouTube link demonstrating the project in action. A visual demo is worth a thousand words.*

![Demo GIF](link_to_your_gif.gif)

---

## ✨ Key Features

-   **Real-time Music Visualization:** Analyzes audio intensity from a microphone module to generate dynamic, synchronized lighting effects.
-   **Pre-programmed Effects:** Includes a smooth, animated "Rainbow Wave" effect.
-   **User Interface:** Features an LCD screen and a rotary encoder for easy navigation and effect selection.

---

## 🛠️ Tech Stack & Components

#### Hardware
-   Microcontroller: **STM32F407VET6**
-   LEDs: **WS2812B** Addressable RGB LED Strip
-   Audio Input: **MAX9814** Microphone Module
-   Display: **ILI9341 TFT LCD** (driven via FSMC)
-   Input: **Rotary Encoder** with push button

#### Software & Tools
-   Language: **C/C++**
-   IDE: **STM32CubeIDE**
-   Libraries: **STM32 HAL, ARM CMSIS**

---

## 🔧 Technical Highlights

This section details the key engineering decisions made to optimize the system's performance.

-   **Non-Blocking LED Control:** The precise 800kHz signal required by WS2812B LEDs is generated using a combination of **Timer PWM and DMA**. This approach completely offloads the CPU, freeing it up to handle complex effect calculations and UI processing in real-time.
-   **Efficient Audio Sampling:** An **ADC** is used in conjunction with **DMA (in Circular Mode)** to continuously sample the audio signal without CPU intervention, ensuring smooth and responsive music visualization.
-   **High-Speed Display Interface:** The **FSMC** (Flexible Static Memory Controller) peripheral is utilized for a parallel interface with the LCD, achieving significantly higher refresh rates compared to a standard SPI connection.
-   **Robust Software Architecture:** The application is built upon a **State Machine** design pattern, which provides a clear, maintainable, and easily extendable structure for managing different operational modes and effects.

---

## 🚀 Future Improvements

-   Implement **FFT (Fast Fourier Transform)** for frequency-based audio analysis (e.g., reacting differently to bass, mids, and treble).
-   Design a custom **PCB** to create a more compact and reliable product.
-   Add **wireless connectivity** (Bluetooth or Wi-Fi) to control the effects from a web or mobile application.

---

## 🙏 Credits & Acknowledgements

-   The LCD driver is based on the excellent **[STM32-ILI9341-FSMC Library by @taburyak](https://github.com/taburyak/STM32-ILI9341-FSMC-Library)**.
