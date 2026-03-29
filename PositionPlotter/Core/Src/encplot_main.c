/*
 * encplot_main.c
 *
 * Simplified AS5600 encoder plotter - polling based
 * No DMA, no timer interrupts. Blocking I2C reads.
 * If a sensor fails to respond, its angle defaults to 0.
 *
 * Compatible with UART protocol expected by main.rs (Sysmic HMI):
 *   Telemetry:  [AA BB ang1_L ang1_H ... ang4_H CHK 0A] (12 bytes)
 *   Diagnostic: [CC DD sensor×4(44B) CHK 0A]             (48 bytes)
 *
 *  Created on: 15 mar 2026
 *      Author: rorosierra
 */

#include "encplot_main.h"
#include "i2c.h"
#include "usart.h"

/* ==========================================
 * AS5600 Register Definitions
 * ========================================== */
#define AS5600_ADDR          (0x36 << 1)
#define AS5600_REG_RAW_ANGLE 0x0C
#define AS5600_REG_ZPOS_H    0x01
#define AS5600_REG_MPOS_H    0x03
#define AS5600_REG_MANG_H    0x05
#define AS5600_REG_STATUS    0x0B
#define AS5600_REG_AGC       0x1A
#define AS5600_REG_MAGNITUDE 0x1B

#define NUM_SENSORS   4
#define I2C_TIMEOUT   10   /* ms – lectura rápida de ángulo */
#define DIAG_TIMEOUT  50   /* ms – lectura lenta de diagnóstico */

/* ==========================================
 * Protocol Frame Definitions
 * ========================================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];    /* 0xAA, 0xBB */
    uint16_t angles[4];    /* Little-endian en STM32 */
    uint8_t  checksum;     /* Suma LSB de los 8 bytes de payload */
    uint8_t  footer;       /* 0x0A */
} TelemetryFrame_t;

typedef struct {
    uint8_t  i2c_ok;
    uint16_t zpos;
    uint16_t mpos;
    uint16_t mang;
    uint8_t  status_reg;
    uint8_t  agc;
    uint16_t magnitude;
} SensorDiag_t;

typedef struct {
    uint8_t      header[2];   /* 0xCC, 0xDD */
    SensorDiag_t sensors[4];
    uint8_t      checksum;
    uint8_t      footer;      /* 0x0A */
} DiagFrame_t;
#pragma pack(pop)

/* ==========================================
 * Private state
 * ========================================== */
static I2C_HandleTypeDef *i2c_bus[NUM_SENSORS];

/* ==========================================
 * AS5600 helpers
 * ========================================== */
static uint16_t as5600_read_angle(uint8_t idx)
{
    uint8_t buf[2] = {0, 0};
    if (HAL_I2C_Mem_Read(i2c_bus[idx], AS5600_ADDR, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT) != HAL_OK) {
        return 0;  /* sensor sin respuesta → default 0 */
    }
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static void as5600_reset_calibration(uint8_t idx)
{
    uint8_t zero[2] = {0, 0};
    HAL_I2C_Mem_Write(i2c_bus[idx], AS5600_ADDR, AS5600_REG_ZPOS_H,
                      I2C_MEMADD_SIZE_8BIT, zero, 2, I2C_TIMEOUT);
    HAL_I2C_Mem_Write(i2c_bus[idx], AS5600_ADDR, AS5600_REG_MPOS_H,
                      I2C_MEMADD_SIZE_8BIT, zero, 2, I2C_TIMEOUT);
    HAL_I2C_Mem_Write(i2c_bus[idx], AS5600_ADDR, AS5600_REG_MANG_H,
                      I2C_MEMADD_SIZE_8BIT, zero, 2, I2C_TIMEOUT);
}

/* ==========================================
 * Telemetry: lee 4 sensores y envía trama
 * ========================================== */
static void telemetry_send(void)
{
    TelemetryFrame_t frame = {
        .header = {0xAA, 0xBB},
        .footer = 0x0A
    };

    for (int i = 0; i < NUM_SENSORS; i++) {
        frame.angles[i] = as5600_read_angle(i);
    }

    uint8_t *p = (uint8_t *)frame.angles;
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += p[i];
    frame.checksum = sum;

    HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(TelemetryFrame_t), 5);
}

/* ==========================================
 * Diagnostic: trama completa 48 bytes
 * ========================================== */
static void diagnostic_send(void)
{
    DiagFrame_t frame = {0};
    frame.header[0] = 0xCC;
    frame.header[1] = 0xDD;
    frame.footer    = 0x0A;

    for (int i = 0; i < NUM_SENSORS; i++) {
        SensorDiag_t *s = &frame.sensors[i];
        uint8_t buf[2];

        if (HAL_I2C_IsDeviceReady(i2c_bus[i], AS5600_ADDR, 3, DIAG_TIMEOUT) != HAL_OK)
            continue;

        s->i2c_ok = 1;
        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_ZPOS_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, DIAG_TIMEOUT);
        s->zpos = (uint16_t)((buf[0] << 8) | buf[1]);

        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_MPOS_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, DIAG_TIMEOUT);
        s->mpos = (uint16_t)((buf[0] << 8) | buf[1]);

        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_MANG_H,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, DIAG_TIMEOUT);
        s->mang = (uint16_t)((buf[0] << 8) | buf[1]);

        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_STATUS,
                         I2C_MEMADD_SIZE_8BIT, &s->status_reg, 1, DIAG_TIMEOUT);
        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_AGC,
                         I2C_MEMADD_SIZE_8BIT, &s->agc, 1, DIAG_TIMEOUT);

        HAL_I2C_Mem_Read(i2c_bus[i], AS5600_ADDR, AS5600_REG_MAGNITUDE,
                         I2C_MEMADD_SIZE_8BIT, buf, 2, DIAG_TIMEOUT);
        s->magnitude = (uint16_t)((buf[0] << 8) | buf[1]);
    }

    uint8_t *p = (uint8_t *)frame.sensors;
    uint8_t sum = 0;
    for (int i = 0; i < (int)sizeof(frame.sensors); i++) sum += p[i];
    frame.checksum = sum;

    HAL_UART_Transmit(&huart3, (uint8_t *)&frame, sizeof(DiagFrame_t), 50);
}

/* ==========================================
 * UART command check (polling, 1 ms timeout)
 * ========================================== */
static void check_command(void)
{
    uint8_t byte;
    if (HAL_UART_Receive(&huart3, &byte, 1, 1) == HAL_OK) {
        if (byte == 0xDD) {
            diagnostic_send();
        }
    }
}

/* ==========================================
 * Public API
 * ========================================== */
void encplot_init(void)
{
    i2c_bus[0] = &hi2c1;
    i2c_bus[1] = &hi2c2;
    i2c_bus[2] = &hi2c3;
    i2c_bus[3] = &hi2c4;

    for (int i = 0; i < NUM_SENSORS; i++) {
        as5600_reset_calibration(i);
    }
    HAL_Delay(10);
}

void encplot_loop(void)
{
    check_command();
    telemetry_send();
    HAL_Delay(10);  /* ~100 Hz */
}




