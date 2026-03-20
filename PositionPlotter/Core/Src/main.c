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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart3; // Ajusta según el UART que uses hacia el PC
extern TIM_HandleTypeDef htim1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_ADDR          (0x36 << 1) // Dirección I2C desplazada
#define AS5600_REG_RAW_ANGLE 0x0C

// --- Registros de calibración del AS5600 ---
#define AS5600_REG_ZPOS_H    0x01    // Zero Position High byte
#define AS5600_REG_MPOS_H    0x03    // Maximum Position High byte
#define AS5600_REG_MANG_H    0x05    // Maximum Angle High byte
#define AS5600_REG_STATUS    0x0B    // Status: MD(5), ML(4), MH(3)
#define AS5600_REG_AGC       0x1A    // Automatic Gain Control
#define AS5600_REG_MAGNITUDE 0x1B    // CORDIC Magnitude High byte
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void AS5600_Reset_Calibration(I2C_HandleTypeDef *hi2c);
void AS5600_Diagnostic(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  // Limpiar calibraciones raras de la RAM de los 4 sensores
  AS5600_Reset_Calibration(&hi2c1);
  AS5600_Reset_Calibration(&hi2c2);
  AS5600_Reset_Calibration(&hi2c3);
  AS5600_Reset_Calibration(&hi2c4);

  // Un pequeño retraso para que el sensor asimile la configuración
  HAL_Delay(10); 

  // Diagnóstico de los 4 AS5600 por UART
  AS5600_Diagnostic();

  // Configurar TIM1 para 10ms: 96MHz / 9600 / 100 = 100 Hz
  htim1.Instance->PSC = 9600 - 1;  // Prescaler -> reloj de 10 kHz
  htim1.Instance->ARR = 100 - 1;   // Period -> 10 ms
  htim1.Instance->CNT = 0;

  // Iniciar la base de tiempo
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  extern volatile uint8_t data_ready;
  extern uint8_t uart_tx_data_bytes[12];  // DataFrame_t es 12 bytes empaquetados
  while (1)
  {
    if (data_ready) {
      data_ready = 0;
      HAL_UART_Transmit(&huart3, uart_tx_data_bytes, 12, 5);
    }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Fuerza a cero los registros ZPOS, MPOS y MANG del AS5600
 *        esto garantiza que el sensor actúe en rango completo [0, 4095] 
 *        sin escalamiento ni offsets indeseados en memoria RAM.
 */
void AS5600_Reset_Calibration(I2C_HandleTypeDef *hi2c) {
    uint8_t zero_buffer[2] = {0x00, 0x00};

    // 1. Limpiar Zero Position (ZPOS en 0x01 y 0x02)
    HAL_I2C_Mem_Write(hi2c, AS5600_ADDR, AS5600_REG_ZPOS_H, I2C_MEMADD_SIZE_8BIT, zero_buffer, 2, 10);
    
    // 2. Limpiar Maximum Position (MPOS en 0x03 y 0x04)
    HAL_I2C_Mem_Write(hi2c, AS5600_ADDR, AS5600_REG_MPOS_H, I2C_MEMADD_SIZE_8BIT, zero_buffer, 2, 10);
    
    // 3. Limpiar Maximum Angle (MANG en 0x05 y 0x06)
    HAL_I2C_Mem_Write(hi2c, AS5600_ADDR, AS5600_REG_MANG_H, I2C_MEMADD_SIZE_8BIT, zero_buffer, 2, 10);
}

/**
 * @brief  Diagnóstico completo de los 4 AS5600 al iniciar.
 *         Imprime por UART3: conexión, ZPOS, MPOS, MANG, STATUS, AGC, MAGNITUDE.
 */
void AS5600_Diagnostic(void) {
    I2C_HandleTypeDef *buses[4] = {&hi2c1, &hi2c2, &hi2c3, &hi2c4};
    char msg[128];
    int len;

    len = snprintf(msg, sizeof(msg),
        "\r\n========== AS5600 DIAGNOSTIC ==========\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);

    for (int i = 0; i < 4; i++) {
        len = snprintf(msg, sizeof(msg), "\r\n--- Sensor %d (I2C%d) ---\r\n", i + 1, i + 1);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);

        // Verificar conexión con IsDeviceReady
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(buses[i], AS5600_ADDR, 3, 50);
        if (status != HAL_OK) {
            len = snprintf(msg, sizeof(msg), "  Estado: NO CONECTADO\r\n");
            HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
            continue;
        }
        len = snprintf(msg, sizeof(msg), "  Estado: CONECTADO\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);

        uint8_t buf[2];

        // Leer ZPOS (0x01-0x02)
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_ZPOS_H, I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
        uint16_t zpos = (buf[0] << 8) | buf[1];

        // Leer MPOS (0x03-0x04)
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MPOS_H, I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
        uint16_t mpos = (buf[0] << 8) | buf[1];

        // Leer MANG (0x05-0x06) - Escala angular
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MANG_H, I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
        uint16_t mang = (buf[0] << 8) | buf[1];

        len = snprintf(msg, sizeof(msg),
            "  ZPOS: %u  |  MPOS: %u  |  MANG: %u\r\n", zpos, mpos, mang);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);

        // Leer STATUS (0x0B)
        uint8_t status_reg = 0;
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, 50);
        uint8_t md = (status_reg >> 5) & 1;  // Magnet Detected
        uint8_t ml = (status_reg >> 4) & 1;  // Magnet too weak
        uint8_t mh = (status_reg >> 3) & 1;  // Magnet too strong

        // Leer AGC (0x1A)
        uint8_t agc = 0;
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_AGC, I2C_MEMADD_SIZE_8BIT, &agc, 1, 50);

        // Leer MAGNITUDE (0x1B-0x1C)
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MAGNITUDE, I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
        uint16_t magnitude = (buf[0] << 8) | buf[1];

        len = snprintf(msg, sizeof(msg),
            "  AGC: %u  |  Magnitud: %u\r\n", agc, magnitude);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);

        // Evaluación del imán
        const char *eval;
        if (!md) {
            eval = "SIN IMAN DETECTADO";
        } else if (ml) {
            eval = "IMAN MUY DEBIL (AGC al maximo)";
        } else if (mh) {
            eval = "IMAN MUY FUERTE (AGC al minimo)";
        } else {
            eval = "OK - Intensidad aceptable";
        }

        len = snprintf(msg, sizeof(msg),
            "  Iman: MD=%u ML=%u MH=%u -> %s\r\n", md, ml, mh, eval);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
    }

    len = snprintf(msg, sizeof(msg),
        "\r\n========================================\r\n\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
