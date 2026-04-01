# Closed-Loop Control Firmware: Digital Power Supply

A high-efficiency, bare-metal firmware implementation for a digitally controlled variable buck converter. This project focuses on precise voltage regulation using an **STM32F103** MCU and a discrete power stage.

## Hardware Specifications
* **Input Voltage:** 19.5V DC
* **Output Voltage:** 0V – 15V (Variable)
* **Max Output Current:** 2.0A
* **Control Architecture:** Synchronous Buck Topology
* **Feedback Sensor:** INA219 (I2C Digital Current/Voltage Monitor)
* **Switching Frequency:** 100 kHz (TIM1 PWM)

##  Control Logic & Firmware Architecture
The firmware utilizes a PI (Proportional-Integral) control algorithm with **Gain Scheduling** to maintain stability across different voltage ranges.

### System Flowchart
![Firmware Logic Flowchart](flowchart.png)

### Key Features
* **Soft-Start Ramp:** Incremental setpoint stepping (100 µV steps) to prevent inrush current.
* **Anti-Windup Clamping:** Prevents integral saturation by limiting the duty cycle between 5% and 90%.
* **Gain Scheduling:** Adjusts $K_p$ values based on the setpoint (Stronger gain for $< 5V$) to optimize transient response.
* **Over-Current Protection (OCP):** Automatic fault trip if current exceeds 2.5A.
* **DMA Integration:** Efficient data transfer from I2C sensors to minimize CPU overhead in the main control loop.

## Project Structure
* `/Core`: Bare-metal source code and header files.
* `/Drivers`: CMSIS and low-level peripheral drivers.
* `*.ioc`: STM32CubeMX configuration file.

## Future Improvements
* Implementation of a Full PID loop (adding Derivative term).
* Integration of an OLED display for real-time V/I telemetry.
* PC-based GUI for remote control via UART.

---
**Developed by Omolo Design**
