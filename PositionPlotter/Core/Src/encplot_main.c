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

// --- Trama de diagnóstico (0xCC 0xDD): 48 bytes totales ---
typedef struct {
    uint8_t  i2c_ok;     // 1 = CONECTADO, 0 = SIN RESPUESTA
    uint16_t zpos;       // Registro ZPOS (0x01-0x02)
    uint16_t mpos;       // Registro MPOS (0x03-0x04)
    uint16_t mang;       // Registro MANG (0x05-0x06)
    uint8_t  status_reg; // STATUS: MD(b5), ML(b4), MH(b3)
    uint8_t  agc;        // AGC (0x1A)
    uint16_t magnitude;  // Magnitude CORDIC (0x1B-0x1C)
} SensorDiag_t;          // 11 bytes por sensor

typedef struct {
    uint8_t      header[2];   // 0xCC, 0xDD
    SensorDiag_t sensors[4];  // 44 bytes (4 × 11)
    uint8_t      checksum;    // LSB de la suma de los 44 bytes de payload
    uint8_t      footer;      // 0x0A
} DiagFrame_t;               // 48 bytes total
#pragma pack(pop)
/* USER CODE END PTD */

/* USER CODE BEGIN PV */
#define AS5600_ADDR          (0x36 << 1) // HAL I2C requiere la dirección desplazada 1 bit
#define AS5600_REG_RAW_ANGLE 0x0C        // Registro base del RAW ANGLE (0x0C High, 0x0D Low)

// Registros de calibración y estado del AS5600 (usados en send_diagnostic_frame)
#define AS5600_REG_ZPOS_H    0x01    // Zero Position High byte
#define AS5600_REG_MPOS_H    0x03    // Maximum Position High byte
#define AS5600_REG_MANG_H    0x05    // Maximum Angle High byte
#define AS5600_REG_STATUS    0x0B    // Status: MD(5), ML(4), MH(3)
#define AS5600_REG_AGC       0x1A    // Automatic Gain Control
#define AS5600_REG_MAGNITUDE 0x1B    // CORDIC Magnitude High byte

// Trama de telemetría lista para transmitir
DataFrame_t uart_tx_data = {
    .header = {0xAA, 0xBB},
    .angles = {0, 0, 0, 0},
    .checksum = 0,
    .footer = 0x0A
};

// Buffers de recepción I2C (un par High/Low por sensor)
volatile uint8_t as5600_rx_buf[4][2];

// Señales entre ISR y main loop
volatile uint8_t data_ready    = 0;
volatile uint8_t uart_rx_byte  = 0;
volatile uint8_t diag_requested = 0;

// Acceso directo a la trama como array de bytes
uint8_t *uart_tx_data_bytes = (uint8_t *)&uart_tx_data;
/* USER CODE END PV */

/* USER CODE BEGIN 4 */

// -----------------------------------------------------------------------
// 1. TIM1 @ 100 Hz: empaqueta los ángulos del ciclo anterior y lanza
//    las 4 nuevas lecturas en paralelo. Cada sensor es independiente:
//    si uno falla su último valor conocido se reenvía sin bloquear.
// -----------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM1) return;

    // Checksum sobre los 8 bytes de payload (ángulos del ciclo anterior)
    uint8_t *p = (uint8_t *)uart_tx_data.angles;
    uint16_t s = 0;
    for (int i = 0; i < 8; i++) s += p[i];
    uart_tx_data.checksum = (uint8_t)(s & 0xFF);
    data_ready = 1;

    // Lanzar las 4 lecturas para el próximo ciclo
    HAL_I2C_Mem_Read_IT(&hi2c1, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)as5600_rx_buf[0], 2);
    HAL_I2C_Mem_Read_IT(&hi2c2, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)as5600_rx_buf[1], 2);
    HAL_I2C_Mem_Read_IT(&hi2c3, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)as5600_rx_buf[2], 2);
    HAL_I2C_Mem_Read_IT(&hi2c4, AS5600_ADDR, AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, (uint8_t *)as5600_rx_buf[3], 2);
}

// -----------------------------------------------------------------------
// 2. I2C RX completado: actualiza únicamente el ángulo del sensor
//    que terminó. Los demás sensores no se ven afectados.
// -----------------------------------------------------------------------
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if      (hi2c->Instance == I2C1) uart_tx_data.angles[0] = ((uint16_t)as5600_rx_buf[0][0] << 8) | as5600_rx_buf[0][1];
    else if (hi2c->Instance == I2C2) uart_tx_data.angles[1] = ((uint16_t)as5600_rx_buf[1][0] << 8) | as5600_rx_buf[1][1];
    else if (hi2c->Instance == I2C3) uart_tx_data.angles[2] = ((uint16_t)as5600_rx_buf[2][0] << 8) | as5600_rx_buf[2][1];
    else if (hi2c->Instance == I2C4) uart_tx_data.angles[3] = ((uint16_t)as5600_rx_buf[3][0] << 8) | as5600_rx_buf[3][1];
}

// -----------------------------------------------------------------------
// 3. Error I2C: soft-reset del periférico para liberar el bus.
//    Sin esto el flag BUSY persiste y Mem_Read_IT retorna HAL_BUSY.
// -----------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    hi2c->Instance->CR1 &= ~I2C_CR1_PE;
    __NOP(); __NOP(); __NOP();
    hi2c->Instance->CR1 |=  I2C_CR1_PE;
    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->Mode      = HAL_I2C_MODE_NONE;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
}

// -----------------------------------------------------------------------
// 4. UART RX: recibe comandos del PC (1 byte). 0xDD → diagnóstico.
// -----------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        if (uart_rx_byte == 0xDD) diag_requested = 1;
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart_rx_byte, 1);
    }
}

// -----------------------------------------------------------------------
// 5. Diagnóstico bajo demanda (0xCC/0xDD, 48 bytes).
//    Usa I2C bloqueante → llamar solo desde el main loop.
//    Los sensores sin respuesta quedan con i2c_ok = 0, resto en 0.
// -----------------------------------------------------------------------
void send_diagnostic_frame(void) {
    I2C_HandleTypeDef *buses[4] = {&hi2c1, &hi2c2, &hi2c3, &hi2c4};
    DiagFrame_t frame = {0};
    frame.header[0] = 0xCC;
    frame.header[1] = 0xDD;
    frame.footer    = 0x0A;

    for (int i = 0; i < 4; i++) {
        SensorDiag_t *s = &frame.sensors[i];
        uint8_t buf[2];
        if (HAL_I2C_IsDeviceReady(buses[i], AS5600_ADDR, 3, 50) != HAL_OK) continue;
        s->i2c_ok = 1;
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_ZPOS_H,    I2C_MEMADD_SIZE_8BIT, buf,            2, 50); s->zpos      = (uint16_t)((buf[0] << 8) | buf[1]);
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MPOS_H,    I2C_MEMADD_SIZE_8BIT, buf,            2, 50); s->mpos      = (uint16_t)((buf[0] << 8) | buf[1]);
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MANG_H,    I2C_MEMADD_SIZE_8BIT, buf,            2, 50); s->mang      = (uint16_t)((buf[0] << 8) | buf[1]);
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_STATUS,    I2C_MEMADD_SIZE_8BIT, &s->status_reg, 1, 50);
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_AGC,       I2C_MEMADD_SIZE_8BIT, &s->agc,        1, 50);
        HAL_I2C_Mem_Read(buses[i], AS5600_ADDR, AS5600_REG_MAGNITUDE, I2C_MEMADD_SIZE_8BIT, buf,            2, 50); s->magnitude = (uint16_t)((buf[0] << 8) | buf[1]);
    }

    uint8_t *p = (uint8_t *)frame.sensors;
    uint16_t sum = 0;
    for (int i = 0; i < (int)sizeof(frame.sensors); i++) sum += p[i];
    frame.checksum = (uint8_t)(sum & 0xFF);

    HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(DiagFrame_t), 50);
}

/* USER CODE END 4 */




