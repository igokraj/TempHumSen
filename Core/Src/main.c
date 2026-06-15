/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NanoEdgeAI.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_uart.h"
#include <stdio.h>
#include <string.h>

#define HTU21D_ADDR (0x40 << 1) // I2C address of the HTU21D sensor
#define Timeout_delay 100
#define BUFFER_SIZE 11
#define FEATURES_PER_SAMPLE 2
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_tx_buffer[64];
uint8_t data[3];
uint8_t temp_cmd = 0xE3;
uint8_t hum_cmd = 0xE5;

typedef struct {
  uint8_t index;
} RingBuffer;

typedef struct {
  float data[BUFFER_SIZE * FEATURES_PER_SAMPLE];
  uint8_t count;
} RetrainingBuffer;

RingBuffer sensor_buffer = {0};        // initial learning (11 samples)
RetrainingBuffer retrain_buffer = {0}; // normal samples for retraining
enum neai_state neai_state_var = NEAI_NOT_INITIALIZED;
uint8_t neai_similarity = 0;

// Timer for retraining every hour
static volatile uint32_t retrain_timer_ms = 0;
#define RETRAINING_INTERVAL_MS (60 * 60 * 1000) // 3600000 ms = 1h
#define SIMILARITY_THRESHOLD 80                 // Threshold for normal samples

void perform_retraining(void) {
  if (retrain_buffer.count < 5) // Min 5 samples
  {
    uint16_t len = sprintf((char *)uart_tx_buffer,
                           "RETRAIN_SKIP: too few samples (%u)\r\n",
                           retrain_buffer.count);
    HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);
    return;
  }

  uint16_t len = sprintf((char *)uart_tx_buffer, "RETRAIN_START: %u samples\r\n",
                         retrain_buffer.count);
  HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);

  // Reinitialize model
  neai_state_var = neai_anomalydetection_init(true); // true = reset
  if (neai_state_var != NEAI_OK) {
    len = sprintf((char *)uart_tx_buffer, "RETRAIN_INIT_ERR:%d\r\n",
                  neai_state_var);
    HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);
    return;
  }

  // Train on collected samples
  for (uint8_t i = 0; i < retrain_buffer.count; i++) {
    float sample[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];
    memcpy(sample, &retrain_buffer.data[i * FEATURES_PER_SAMPLE],
           FEATURES_PER_SAMPLE * sizeof(float));

    neai_state_var = neai_anomalydetection_learn(sample);
    if (neai_state_var != NEAI_LEARNING_IN_PROGRESS &&
        neai_state_var != NEAI_LEARNING_DONE) {
      len = sprintf((char *)uart_tx_buffer, "RETRAIN_LEARN_ERR:%d\r\n",
                    neai_state_var);
      HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);
      return;
    }
  }

  // Finalize learning - model has been trained
  retrain_buffer.count = 0; // Clear buffer

  len = sprintf((char *)uart_tx_buffer, "RETRAIN_DONE\r\n");
  HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);
}

void add_sensor_data(float temp, float hum) {
  float sample[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER] = {temp, hum};
  uint8_t *msg_ptr = uart_tx_buffer;
  int msg_len = 0;

  // Phase 1: Initial learning (11 samples without filtering)
  if (sensor_buffer.index < BUFFER_SIZE) {
    neai_state_var = neai_anomalydetection_learn(sample);
    sensor_buffer.index++;

    if (neai_state_var == NEAI_LEARNING_IN_PROGRESS) {
      msg_len = sprintf((char *)msg_ptr, "LEARNING:%u/%u\r\n",
                        sensor_buffer.index, BUFFER_SIZE);
    } else if (neai_state_var == NEAI_LEARNING_DONE) {
      msg_len = sprintf((char *)msg_ptr, "LEARNING_DONE\r\n");
    } else {
      msg_len = sprintf((char *)msg_ptr, "LEARN_ERR:%d\r\n", neai_state_var);
    }
  }
  // Phase 2: Detection + filtering for retraining
  else {
    neai_state_var = neai_anomalydetection_detect(sample, &neai_similarity);
    if (neai_state_var == NEAI_OK) {
      // Filter - if similarity > SIMILARITY_THRESHOLD, add to retrain buffer
      if (neai_similarity > SIMILARITY_THRESHOLD) {
        if (retrain_buffer.count < BUFFER_SIZE) {
          memcpy(
              &retrain_buffer.data[retrain_buffer.count * FEATURES_PER_SAMPLE],
              sample, FEATURES_PER_SAMPLE * sizeof(float));
          retrain_buffer.count++;
          msg_len = sprintf((char *)msg_ptr, "ANOMALY:%u [NORMAL]\r\n",
                            neai_similarity);
        } else {
          msg_len =
              sprintf((char *)msg_ptr, "ANOMALY:%u [NORMAL-BUFFER FULL]\r\n",
                      neai_similarity);
        }
      } else {
        msg_len = sprintf((char *)msg_ptr, "ANOMALY:%u [ANOMALY]\r\n",
                          neai_similarity);
      }
    } else {
      msg_len = sprintf((char *)msg_ptr, "DETECT_ERR:%d\r\n", neai_state_var);
    }
  }

  if (msg_len > 0 && msg_len < 64) {
    HAL_UART_Transmit(&huart2, uart_tx_buffer, msg_len, 100);
  }
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  neai_state_var = neai_anomalydetection_init(false);
  if (neai_state_var != NEAI_OK) {
    uint16_t len =
        sprintf((char *)uart_tx_buffer, "NEAI_INIT_ERR:%d\r\n", neai_state_var);
    HAL_UART_Transmit(&huart2, uart_tx_buffer, len, 100);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // Check if it's time to perform retraining
    if (retrain_timer_ms >= RETRAINING_INTERVAL_MS) {
      retrain_timer_ms = 0; // Reset timer
      perform_retraining();
    }

    if (HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR, &temp_cmd, 1,
                                Timeout_delay) == HAL_OK) {
      HAL_Delay(50);
      if (HAL_I2C_Master_Receive(&hi2c1, HTU21D_ADDR, data, 3, Timeout_delay) ==
          HAL_OK) {
           int raw_temp =
             ((data[0] << 8) | data[1]) & 0xFFFC; // mask status bits
        float temperature = -46.85f + (175.72f * raw_temp / 65536.0f);

        if (HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR, &hum_cmd, 1,
                                    Timeout_delay) == HAL_OK) {
          HAL_Delay(50);
          if (HAL_I2C_Master_Receive(&hi2c1, HTU21D_ADDR, data, 3,
                                     Timeout_delay) == HAL_OK) {
               int raw_hum =
                 ((data[0] << 8) | data[1]) & 0xFFFC; // mask status bits
            float humidity = -6.0f + (125.0f * raw_hum / 65536.0f);

            add_sensor_data(temperature, humidity);

            sprintf((char *)uart_tx_buffer,
                    "Temp: %.2f *C, Humidity: %.2f %%\r\n", temperature,
                    humidity);
            HAL_UART_Transmit(&huart2, uart_tx_buffer,
                              strlen((char *)uart_tx_buffer), 500);
          }
        }
      }
    }
    HAL_Delay(2000); // Wait before the next reading

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    // Toggle LED
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  if (htim->Instance == TIM2) {
    /* Increment retraining timer (increment every 1ms, TIM2 is
    configured for 1ms period) */ 
    retrain_timer_ms++;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
