/*
 * encplot_main.c
 *
 *  Created on: 15 mar 2026
 *      Author: rorosierra
 */

#include "main.h"
#include "i2c.h"
#include "usart.h"

/* USER CODE BEGIN PTD */
// Estructura empaquetada para evitar padding y coincidir exactamente con 12 bytes
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];    // 0xAA, 0xBB
    uint16_t angles[4];    // Ángulos de los 4 sensores (Little Endian en STM32)
    uint8_t  checksum;     // Suma LSB de los 8 bytes de payload
    uint8_t  footer;       // 0x0A
} DataFrame_t;
#pragma pack(pop)
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
#define AS5600_ADDR          (0x36 << 1) // HAL I2C requiere la dirección desplazada 1 bit
#define AS5600_REG_RAW_ANGLE 0x0C        // Registro base del RAW ANGLE (0x0C High, 0x0D Low)

// Variables de datos
DataFrame_t uart_tx_data = {
    .header = {0xAA, 0xBB},
    .angles = {0, 0, 0, 0},
    .checksum = 0,
    .footer = 0x0A
};

// Buffers crudos para DMA de cada bus I2C (2 bytes c/u: High Byte, Low Byte)
uint8_t as5600_rx_buf[4][2];

// Flags de sincronización y control de errores
volatile uint8_t i2c_cplt_flags = 0;
volatile uint8_t data_ready = 0;  // Flag para que el main loop envíe por UART

// Alias para acceder como bytes desde main.c
uint8_t *uart_tx_data_bytes = (uint8_t *)&uart_tx_data;
/* USER CODE END PV */

/* USER CODE BEGIN 4 */

// 1. Callback de Timer: Se ejecuta cada 10ms e inicia las 4 transferencias I2C simultáneas
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        i2c_cplt_flags = 0; // Reiniciar flags en cada ciclo

        // Disparar las 4 lecturas. Leemos 2 bytes consecutivos (0x0C y 0x0D)
        HAL_I2C_Mem_Read_DMA(&hi2c1, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, as5600_rx_buf[0], 2);
        HAL_I2C_Mem_Read_DMA(&hi2c2, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, as5600_rx_buf[1], 2);
        HAL_I2C_Mem_Read_DMA(&hi2c3, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, as5600_rx_buf[2], 2);
        HAL_I2C_Mem_Read_DMA(&hi2c4, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, as5600_rx_buf[3], 2);
    }
}

// 2. Callback de I2C: Se llama cada vez que un I2C termina de guardar sus 2 bytes por DMA
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) i2c_cplt_flags |= (1 << 0);
    if (hi2c->Instance == I2C2) i2c_cplt_flags |= (1 << 1);
    if (hi2c->Instance == I2C3) i2c_cplt_flags |= (1 << 2);
    if (hi2c->Instance == I2C4) i2c_cplt_flags |= (1 << 3);

    // Si los 4 sensores respondieron correctamente (0b00001111 = 0x0F)
    if (i2c_cplt_flags == 0x0F) {
        i2c_cplt_flags = 0; // Evitar reentrada

        // AS5600 envía [0]: High Byte, [1]: Low Byte. Ensamblamos para el struct
        uart_tx_data.angles[0] = (as5600_rx_buf[0][0] << 8) | as5600_rx_buf[0][1];
        uart_tx_data.angles[1] = (as5600_rx_buf[1][0] << 8) | as5600_rx_buf[1][1];
        uart_tx_data.angles[2] = (as5600_rx_buf[2][0] << 8) | as5600_rx_buf[2][1];
        uart_tx_data.angles[3] = (as5600_rx_buf[3][0] << 8) | as5600_rx_buf[3][1];

        // Calcular Checksum matemático (solo sobre los 8 bytes del payload)
        uint8_t *payload_ptr = (uint8_t *)uart_tx_data.angles;
        uint16_t sum = 0;
        for (int i = 0; i < 8; i++) {
            sum += payload_ptr[i];
        }
        uart_tx_data.checksum = (uint8_t)(sum & 0xFF);

        // Señalizar al main loop que los datos están listos
        data_ready = 1;
    }
}

// 3. (OPCIONAL PERO CRÍTICO): Prevención de bloqueos si un sensor se desconecta.
// Si un I2C falla (NACK), este bus no emite RxCplt, por lo que bloquea a los demás.
// Forzamos la bandera de completado con alerta para no trabar el ciclo entero.
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) i2c_cplt_flags |= (1 << 0);
    if (hi2c->Instance == I2C2) i2c_cplt_flags |= (1 << 1);
    if (hi2c->Instance == I2C3) i2c_cplt_flags |= (1 << 2);
    if (hi2c->Instance == I2C4) i2c_cplt_flags |= (1 << 3);
}

/* USER CODE END 4 */




