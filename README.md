# TempHumSen

A STM32F4 sensor project for temperature and humidity monitoring with integrated NanoEdge AI anomaly detection.

<img width="767" height="564" alt="image" src="https://github.com/user-attachments/assets/68503a55-083c-457f-a80a-0b787d7a62d4" />


## Project Overview

This project reads temperature and humidity from an HTU21D sensor over I2C, performs anomaly detection using the NanoEdge AI library, and reports results over UART. An on-board LED is used as a 2-second heartbeat indicator that matches the measurement interval.

## How it Works

- The STM32F446 MCU initializes I2C, UART, TIM1, TIM2, and the NanoEdge AI model.
- Every 2 seconds, the firmware reads the HTU21D temperature and humidity values.
- TIM1 provides the LED heartbeat for each measurement cycle.
- TIM2 increments the retraining timer used to trigger periodic model retraining.
- The data is sent to the AI algorithm for either learning or anomaly detection.
- The on-board LED toggles every measurement cycle to show that a new data sample was taken.

## LED Indicator

The LED is controlled by `TIM1` and toggles every 2 seconds. This provides a visual heartbeat that coincides with the measurement interval and confirms the sensor measurement cycle.

## Retraining Timer

`TIM2` is used to increment `retrain_timer_ms` on each timer interrupt, which lets the firmware trigger periodic retraining of the AI model without blocking the main loop.

## AI Implementation

The project uses the NanoEdge AI anomaly detection API:

- Initial training phase collects the first 11 samples without filtering.
- After the initial learning phase, the model switches to detection mode.
- Each new sample is compared against the learned model using similarity scoring.
- If the similarity score is above the normal threshold, the sample is considered normal and stored in a retraining buffer.
- If the similarity score is below the threshold, the sample is flagged as an anomaly.
- Periodically, the firmware can retrain the model using accumulated normal samples, driven by the `TIM2` retraining timer.

### NanoEdge AI Flow

1. `neai_anomalydetection_init(false)` initializes the anomaly detection engine.
2. `neai_anomalydetection_learn(sample)` is called during the initial learning phase and during retraining.
3. `neai_anomalydetection_detect(sample, &neai_similarity)` is used once the model is trained to evaluate new samples.
4. The similarity score determines whether the sample is normal or anomalous.

## Requirements

- STM32CubeMX
- arm-none-eabi-gcc 14.3
- NanoEdgeAI lib

### Hardware

- STM32F4 series MCU (STM32F446 target in this project)
- HTU21D temperature and humidity sensor connected to I2C1
- A UART interface for debug output (USART2 used in this project)
- On-board LED connected to `LD2_GPIO_Port` and `LD2_Pin`

### Software

- GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- STM32CubeMX / STM32CubeIDE compatible HAL libraries
- `NanoEdgeAI` library included in `nanoedge_ai_lib/`
- CMake build files already provided in the project

## Build and Flash

1. Configure your toolchain for the STM32F446 target.
2. Build the project using the provided CMake configuration.
3. Flash the resulting binary to the STM32 board.

## Notes

- The firmware uses a 2-second measurement cycle.
- The LED heartbeat is synchronized with the measurement interval.
- The AI model starts with a cold-learning phase and then performs detection on new samples.
- Normal measurements are buffered for retraining to keep the model updated over time.
