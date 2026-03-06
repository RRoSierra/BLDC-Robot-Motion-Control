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
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/*
 * CONFIGURACIÓN CUBEMX COMPLETADA:
 * - TIM1: PSC=639, ARR=999 → 100Hz. Update Interrupt habilitado.
 * - PC13: GPIO_EXTI13 Falling Edge. EXTI15_10_IRQn habilitado.
 * - FatFs: Middleware FATFS User-defined. _USE_STRFUNC=2 en ffconf.h.
 * - SPI1: DataSize corregido a 8 bits en spi.c (USER CODE SPI1_Init 2).
 * - PB14 (LD3): Init manual en USER CODE BEGIN 2 (no en CubeMX).
 * - PA4 (SD_CS): Init manual en user_diskio.c.
 */
/* fatfs.h y tim.h ya incluidos arriba por CubeMX; solo añadir stdlib */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Estructura para el estado de cada encoder magnético AS5600 */
typedef struct {
    I2C_HandleTypeDef *hi2c;      /* Handle del bus I2C asociado              */
    uint16_t raw_angle;           /* Ángulo raw actual (0-4095)               */
    uint16_t prev_raw_angle;      /* Ángulo raw anterior (cálculo velocidad)  */
    float    position_deg;        /* Posición convertida a grados (0 - 360)   */
    float    velocity_dps;        /* Velocidad angular [grados/s]             */
    int16_t  offset;              /* Offset de calibración [counts] (opcional)*/
} AS5600_t;

/* Estructura para una muestra completa de telemetría */
typedef struct {
    uint32_t timestamp_ms;        /* Marca de tiempo [ms] (HAL_GetTick)       */
    float    position[4];         /* Posiciones de los 4 encoders [grados]    */
    float    velocity[4];         /* Velocidades de los 4 encoders [grados/s] */
} TelemetryData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* --- Definiciones del encoder AS5600 --- */
#define AS5600_I2C_ADDR          (0x36 << 1)         /* Dir. I2C left-shifted para HAL */
#define AS5600_REG_RAW_ANGLE     0x0C                 /* Registro Raw Angle High        */
#define AS5600_RESOLUTION        4096                 /* Resolución 12 bits             */
#define AS5600_HALF_RESOLUTION   2048                 /* Mitad (detección wrap-around)   */
#define DEG_PER_COUNT            (360.0f / 4096.0f)   /* Grados por count               */
#define NUM_ENCODERS             4

/* --- Configuración de muestreo y buffer --- */
#define SAMPLE_RATE_HZ           100.0f               /* Frecuencia del TIM1 [Hz]       */
#define TELEM_BUF_SIZE           50                   /* Tamaño del buffer de telemetría*/
#define TELEM_FLUSH_THRESHOLD    25                   /* Umbral para volcado a SD       */

/* --- LED de error (LD3 rojo en NUCLEO-H755ZI-Q = PB14) --- */
#define LED_ERROR_PORT           GPIOB
#define LED_ERROR_PIN            GPIO_PIN_14

/* --- Anti-rebote del botón [ms] --- */
#define DEBOUNCE_TIME_MS         250

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- Encoders AS5600 ---- */
static AS5600_t encoders[NUM_ENCODERS];

/* ---- Buffer de telemetría (doble buffer: ISR escribe, main copia y vuelca) ---- */
static TelemetryData_t telem_buffer[TELEM_BUF_SIZE];    /* Buffer principal (ISR)    */
static TelemetryData_t telem_sd_copy[TELEM_BUF_SIZE];   /* Copia segura para SD      */
static volatile uint16_t telem_write_idx   = 0;         /* Índice de escritura (ISR)  */
static volatile uint8_t  buffer_ready_flag = 0;         /* Flag: datos listos para SD */
static uint16_t telem_flush_count          = 0;         /* Nº muestras a volcar       */

/* ---- Control de grabación y botón ---- */
static volatile uint8_t is_recording      = 0;          /* 1=grabando, 0=detenido     */
static volatile uint8_t button_event_flag = 0;          /* Flag: botón presionado      */
static uint32_t last_button_tick          = 0;          /* Tick anti-rebote            */

/* ---- FatFs / Tarjeta SD ---- */
static FATFS    sd_fatfs;                                /* Objeto sistema de archivos  */
static FIL      sd_file;                                 /* Archivo CSV abierto         */
static uint16_t log_file_number = 0;                     /* Número secuencial del log   */
static uint8_t  sd_mounted      = 0;                     /* 1=SD montada correctamente  */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Funciones del AS5600 */
static HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_t *enc, uint16_t *raw_angle);
static void     AS5600_UpdateVelocity(AS5600_t *enc);
static void     AS5600_InitAll(void);

/* Funciones de la tarjeta SD / FatFs */
static uint8_t  SD_MountAndCheck(void);
static uint8_t  SD_OpenNewLogFile(void);
static void     SD_WriteBufferToFile(uint16_t count);
static void     SD_FlushAndClose(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==========================================================================
 *  FUNCIONES DE LECTURA DEL AS5600
 * ========================================================================== */

/**
 * @brief  Lee el ángulo raw (registro 0x0C, 2 bytes) de un encoder AS5600.
 * @param  enc       Puntero a la estructura del encoder.
 * @param  raw_angle Puntero donde se almacenará el valor (0-4095).
 * @retval HAL_StatusTypeDef
 * @note   Lectura bloqueante rápida (~200µs a 100kHz I2C, timeout 2ms).
 *         Cada encoder tiene su propio bus I2C → sin conflicto entre lecturas.
 */
static HAL_StatusTypeDef AS5600_ReadRawAngle(AS5600_t *enc, uint16_t *raw_angle)
{
    uint8_t buf[2] = {0};
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(enc->hi2c, AS5600_I2C_ADDR,
                              AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT,
                              buf, 2, 2);
    if (status == HAL_OK)
    {
        /* Registro 0x0C: bits [11:8], Registro 0x0D: bits [7:0] */
        *raw_angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }
    return status;
}

/**
 * @brief  Calcula la velocidad angular con manejo correcto de wrap-around.
 * @param  enc Puntero a la estructura del encoder.
 * @note   El wrap-around ocurre al cruzar el punto 0/4095:
 *         - Si delta > +2048 → giro negativo cruzando 0 (ej: 4090→5)
 *         - Si delta < -2048 → giro positivo cruzando 4095 (ej: 5→4090)
 */
static void AS5600_UpdateVelocity(AS5600_t *enc)
{
    int32_t delta = (int32_t)enc->raw_angle - (int32_t)enc->prev_raw_angle;

    /* Corrección del wrap-around */
    if (delta > AS5600_HALF_RESOLUTION)
    {
        delta -= AS5600_RESOLUTION;
    }
    else if (delta < -AS5600_HALF_RESOLUTION)
    {
        delta += AS5600_RESOLUTION;
    }

    /* Vel [°/s] = delta_counts × (°/count) × freq_muestreo */
    enc->velocity_dps = (float)delta * DEG_PER_COUNT * SAMPLE_RATE_HZ;

    /* Posición actual en grados (0° a ~360°) */
    enc->position_deg = (float)enc->raw_angle * DEG_PER_COUNT;
}

/**
 * @brief  Inicializa los 4 encoders: asigna buses I2C y realiza lectura inicial.
 */
static void AS5600_InitAll(void)
{
    encoders[0].hi2c = &hi2c1;
    encoders[1].hi2c = &hi2c2;
    encoders[2].hi2c = &hi2c3;
    encoders[3].hi2c = &hi2c4;

    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        encoders[i].offset       = 0;
        encoders[i].velocity_dps = 0.0f;
        encoders[i].position_deg = 0.0f;

        /* Lectura inicial para tener un prev_raw_angle válido */
        uint16_t raw = 0;
        if (AS5600_ReadRawAngle(&encoders[i], &raw) == HAL_OK)
        {
            encoders[i].raw_angle      = raw;
            encoders[i].prev_raw_angle = raw;
        }
        else
        {
            encoders[i].raw_angle      = 0;
            encoders[i].prev_raw_angle = 0;
        }
    }
}

/* ==========================================================================
 *  FUNCIONES DE LA TARJETA SD / FatFs
 * ========================================================================== */

/**
 * @brief  Monta el sistema de archivos FAT en la tarjeta SD.
 * @retval 1=éxito, 0=error (enciende LED de error).
 */
static uint8_t SD_MountAndCheck(void)
{
    FRESULT fres = f_mount(&sd_fatfs, "", 1);  /* "" = drive 0, montaje inmediato */
    if (fres != FR_OK)
    {
        HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
        sd_mounted = 0;
        return 0;
    }
    HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET);
    sd_mounted = 1;
    return 1;
}

/**
 * @brief  Crea/abre un nuevo archivo CSV con nombre secuencial.
 * @retval 1=éxito, 0=error.
 */
static uint8_t SD_OpenNewLogFile(void)
{
    FRESULT fres;
    char filename[16];

    /* Buscar el siguiente número de archivo disponible */
    do
    {
        log_file_number++;
        if (log_file_number > 999) log_file_number = 1;
        snprintf(filename, sizeof(filename), "log_%03u.csv", log_file_number);
        fres = f_stat(filename, NULL);
    } while (fres == FR_OK);

    /* Crear el archivo */
    fres = f_open(&sd_file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
        return 0;
    }

    /* Escribir cabecera del CSV (requiere FF_USE_STRFUNC >= 1 en ffconf.h) */
    f_printf(&sd_file,
             "timestamp_ms,"
             "pos1_deg,pos2_deg,pos3_deg,pos4_deg,"
             "vel1_dps,vel2_dps,vel3_dps,vel4_dps\n");
    f_sync(&sd_file);

    return 1;
}

/**
 * @brief  Escribe 'count' muestras del buffer secundario al archivo CSV.
 * @param  count Número de muestras a escribir desde telem_sd_copy[].
 * @note   Formateo entero (×100) → sin necesidad de '-u _printf_float'.
 *         Ej línea: "12345,180.50,90.25,270.75,0.00,500.00,-120.30,0.00,80.10"
 */
static void SD_WriteBufferToFile(uint16_t count)
{
    if (!sd_mounted || count == 0) return;

    char line[160];
    UINT bw;

    for (uint16_t i = 0; i < count; i++)
    {
        TelemetryData_t *s = &telem_sd_copy[i];
        int len = 0;

        /* Timestamp */
        len += sprintf(&line[len], "%lu,", (unsigned long)s->timestamp_ms);

        /* 4 posiciones [grados] */
        for (uint8_t j = 0; j < 4; j++)
        {
            int32_t sc = (int32_t)(s->position[j] * 100.0f);
            if (sc < 0) { line[len++] = '-'; sc = -sc; }
            len += sprintf(&line[len], "%ld.%02ld,",
                           (long)(sc / 100), (long)(sc % 100));
        }

        /* 4 velocidades [grados/s] */
        for (uint8_t j = 0; j < 4; j++)
        {
            int32_t sc = (int32_t)(s->velocity[j] * 100.0f);
            if (sc < 0) { line[len++] = '-'; sc = -sc; }
            len += sprintf(&line[len], "%ld.%02ld",
                           (long)(sc / 100), (long)(sc % 100));
            if (j < 3) line[len++] = ',';
        }

        line[len++] = '\n';

        if (f_write(&sd_file, line, (UINT)len, &bw) != FR_OK)
        {
            HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
            break;
        }
    }
    f_sync(&sd_file);
}

/**
 * @brief  Vuelca datos remanentes y cierra el archivo CSV.
 */
static void SD_FlushAndClose(void)
{
    /* Capturar datos remanentes en sección crítica */
    __disable_irq();
    uint16_t remaining = telem_write_idx;
    if (remaining > 0)
    {
        memcpy(telem_sd_copy, telem_buffer,
               remaining * sizeof(TelemetryData_t));
    }
    telem_write_idx   = 0;
    buffer_ready_flag = 0;
    __enable_irq();

    if (remaining > 0)
    {
        SD_WriteBufferToFile(remaining);
    }
    f_close(&sd_file);
}

/* ==========================================================================
 *  CALLBACKS DE HAL (interrupción de botón y timer)
 * ========================================================================== */

/**
 * @brief  Callback de interrupción externa (botón de usuario PC13).
 * @note   Solo señaliza al main loop con anti-rebote por software.
 *         El manejo de FatFs se realiza en el while(1), NO aquí.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_button_tick) >= DEBOUNCE_TIME_MS)
        {
            last_button_tick = now;
            button_event_flag = 1;
        }
    }
}

/**
 * @brief  Callback del timer TIM1 (100Hz). Muestrea los 4 encoders.
 * @note   NO escribe en la SD. Solo llena el buffer y señaliza al main loop.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) return;
    if (!is_recording) return;
    if (telem_write_idx >= TELEM_BUF_SIZE) return;  /* Buffer lleno, descartar */

    /* Leer los 4 encoders y calcular velocidad */
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        encoders[i].prev_raw_angle = encoders[i].raw_angle;
        uint16_t raw = 0;
        if (AS5600_ReadRawAngle(&encoders[i], &raw) == HAL_OK)
        {
            encoders[i].raw_angle = raw;
        }
        /* Si falla la lectura, se conserva el valor anterior */
        AS5600_UpdateVelocity(&encoders[i]);
    }

    /* Almacenar muestra en el buffer */
    TelemetryData_t *sample = &telem_buffer[telem_write_idx];
    sample->timestamp_ms = HAL_GetTick();
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
    {
        sample->position[i] = encoders[i].position_deg;
        sample->velocity[i] = encoders[i].velocity_dps;
    }
    telem_write_idx++;

    /* Señalizar al main loop cuando se alcanza el umbral */
    if (telem_write_idx >= TELEM_FLUSH_THRESHOLD && !buffer_ready_flag)
    {
        buffer_ready_flag = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* Configurar PB14 (LD3 rojo) como GPIO Output para LED de error.
   * El clock de GPIOB ya está habilitado por MX_GPIO_Init (pins I2C). */
  {
    GPIO_InitTypeDef gpio_led = {0};
    gpio_led.Pin   = LED_ERROR_PIN;
    gpio_led.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_led.Pull  = GPIO_NOPULL;
    gpio_led.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_ERROR_PORT, &gpio_led);
    HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET);
  }

  /* Inicializar estructuras de los 4 encoders AS5600 */
  AS5600_InitAll();

  /* Intentar montar la tarjeta SD */
  if (!SD_MountAndCheck())
  {
    /* La SD no se pudo montar. LED de error encendido.
     * El sistema continúa pero la grabación no estará disponible. */
  }

  /* Iniciar TIM1 con interrupción de Update a 100Hz */
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- Manejo del evento de botón (inicio/parada de grabación) ---- */
    if (button_event_flag)
    {
      button_event_flag = 0;

      if (!is_recording)
      {
        /* Iniciar grabación: abrir nuevo archivo CSV */
        if (sd_mounted && SD_OpenNewLogFile())
        {
          __disable_irq();
          telem_write_idx   = 0;
          buffer_ready_flag = 0;
          __enable_irq();

          is_recording = 1;
          HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_RESET);
        }
        else
        {
          /* Error al abrir archivo: señalizar con LED */
          HAL_GPIO_WritePin(LED_ERROR_PORT, LED_ERROR_PIN, GPIO_PIN_SET);
        }
      }
      else
      {
        /* Detener grabación: vaciar buffer remanente y cerrar archivo */
        is_recording = 0;
        SD_FlushAndClose();
      }
    }

    /* ---- Volcado del buffer de telemetría a la SD ---- */
    if (buffer_ready_flag && is_recording)
    {
      /* Sección crítica: copiar datos y reiniciar buffer para la ISR */
      __disable_irq();
      telem_flush_count = telem_write_idx;
      memcpy(telem_sd_copy, telem_buffer,
             telem_flush_count * sizeof(TelemetryData_t));
      telem_write_idx   = 0;
      buffer_ready_flag = 0;
      __enable_irq();

      /* Escribir datos copiados al CSV (fuera de sección crítica) */
      SD_WriteBufferToFile(telem_flush_count);
    }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
