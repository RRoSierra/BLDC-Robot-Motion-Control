/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
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

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "spi.h"

/* ===========================================================================
 * Driver SPI para tarjeta SD sobre SPI1.
 * Pin Chip Select: PA4 (ajustar SD_CS_PORT/PIN si se usa otro pin).
 * =========================================================================== */

/* ---- Pin Chip Select ---- */
#define SD_CS_PORT    GPIOA
#define SD_CS_PIN     GPIO_PIN_4
#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET)

/* ---- Comandos SPI-SD ---- */
#define SD_CMD0     0             /* GO_IDLE_STATE              */
#define SD_CMD1     1             /* SEND_OP_COND (MMC)         */
#define SD_CMD8     8             /* SEND_IF_COND               */
#define SD_CMD9     9             /* SEND_CSD                   */
#define SD_CMD12    12            /* STOP_TRANSMISSION          */
#define SD_CMD16    16            /* SET_BLOCKLEN               */
#define SD_CMD17    17            /* READ_SINGLE_BLOCK          */
#define SD_CMD18    18            /* READ_MULTIPLE_BLOCK        */
#define SD_CMD24    24            /* WRITE_BLOCK                */
#define SD_CMD25    25            /* WRITE_MULTIPLE_BLOCK       */
#define SD_CMD55    55            /* APP_CMD                    */
#define SD_CMD58    58            /* READ_OCR                   */
#define SD_ACMD41   (0x80 | 41)  /* SD_SEND_OP_COND (auto CMD55) */

/* ---- Tipos de tarjeta ---- */
#define SD_TYPE_NONE  0x00
#define SD_TYPE_SD1   0x01  /* SD v1 (byte addressing)  */
#define SD_TYPE_SD2   0x02  /* SD v2 (byte addressing)  */
#define SD_TYPE_SDHC  0x04  /* SDHC/SDXC (block addr)   */
#define SD_TYPE_MMC   0x08  /* MMC                      */

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t sd_type = SD_TYPE_NONE;
static uint8_t sd_ff_buf[520]; /* Buffer de 0xFF constante para SPI RX */

/* ===================== Funciones auxiliares SPI ========================== */

/** Transmitir un byte por SPI1. */
static inline void SD_SPI_TxByte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

/** Recibir un byte por SPI1 (envía 0xFF como clock). */
static inline uint8_t SD_SPI_RxByte(void)
{
    uint8_t tx = 0xFF, rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/** Recibir múltiples bytes. Envía 0xFF (desde sd_ff_buf) mientras recibe. */
static void SD_SPI_RxMulti(uint8_t *data, uint16_t len)
{
    HAL_SPI_TransmitReceive(&hspi1, sd_ff_buf, data, len, HAL_MAX_DELAY);
}

/** Transmitir múltiples bytes. */
static void SD_SPI_TxMulti(const uint8_t *data, uint16_t len)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, HAL_MAX_DELAY);
}

/** SPI lento (~312 kHz) para inicialización SD (requiere ≤400 kHz). */
static void SD_SPI_SetSlow(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    HAL_SPI_Init(&hspi1);
}

/** SPI rápido (~20 MHz) para operación normal. */
static void SD_SPI_SetFast(void)
{
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    HAL_SPI_Init(&hspi1);
}

/** Esperar a que la SD esté lista (MISO=0xFF). Retorna 1=lista, 0=timeout. */
static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    do {
        if (SD_SPI_RxByte() == 0xFF) return 1;
    } while ((HAL_GetTick() - start) < timeout_ms);
    return 0;
}

/** Deseleccionar SD (CS HIGH + byte extra para liberar MISO). */
static void SD_Deselect(void)
{
    SD_CS_HIGH();
    SD_SPI_RxByte();
}

/** Seleccionar SD (CS LOW + esperar ready). Retorna 1=ok, 0=timeout. */
static uint8_t SD_Select(void)
{
    SD_CS_LOW();
    SD_SPI_RxByte();
    if (SD_WaitReady(500)) return 1;
    SD_Deselect();
    return 0;
}

/**
 * @brief  Enviar comando SPI-SD y obtener respuesta R1.
 * @param  cmd  Índice del comando (0-63). Si bit 0x80 → ACMD (CMD55 previo).
 * @param  arg  Argumento de 32 bits.
 * @retval Byte R1 (bit7=1 indica timeout).
 */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    uint8_t res, n;
    uint8_t frame[6];

    /* Manejo de ACMDs: enviar CMD55 primero */
    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        res = SD_SendCmd(SD_CMD55, 0);
        if (res > 1) return res;
    }

    /* Deseleccionar/reseleccionar (excepto CMD12) */
    if (cmd != SD_CMD12)
    {
        SD_Deselect();
        if (!SD_Select()) return 0xFF;
    }

    /* Construir trama del comando */
    frame[0] = 0x40 | cmd;
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)(arg);
    frame[5] = (cmd == SD_CMD0) ? 0x95 : (cmd == SD_CMD8) ? 0x87 : 0x01;

    SD_SPI_TxMulti(frame, 6);

    if (cmd == SD_CMD12) SD_SPI_RxByte(); /* Skip stuff byte */

    /* Esperar respuesta R1 (máximo 10 bytes) */
    n = 10;
    do { res = SD_SPI_RxByte(); } while ((res & 0x80) && --n);

    return res;
}

/** Recibir bloque de datos (esperar token 0xFE + datos + CRC). Retorna 1=ok. */
static uint8_t SD_RxDataBlock(uint8_t *buff, uint16_t len)
{
    uint8_t token;
    uint32_t start = HAL_GetTick();

    do { token = SD_SPI_RxByte(); }
    while (token == 0xFF && (HAL_GetTick() - start) < 200);

    if (token != 0xFE) return 0;

    SD_SPI_RxMulti(buff, len);
    SD_SPI_RxByte(); /* CRC hi */
    SD_SPI_RxByte(); /* CRC lo */
    return 1;
}

/** Transmitir bloque de datos (token + 512B + CRC dummy). Retorna 1=aceptado. */
static uint8_t SD_TxDataBlock(const uint8_t *buff, uint8_t token)
{
    if (!SD_WaitReady(500)) return 0;

    SD_SPI_TxByte(token);
    if (token == 0xFD) return 1; /* Stop token: sin datos */

    SD_SPI_TxMulti(buff, 512);
    SD_SPI_TxByte(0xFF); /* CRC dummy */
    SD_SPI_TxByte(0xFF);

    return ((SD_SPI_RxByte() & 0x1F) == 0x05) ? 1 : 0;
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    GPIO_InitTypeDef gpio_cs = {0};
    uint8_t n, ocr[4], cmd_poll = 0;
    uint32_t start;

    /* 1) Configurar pin CS como salida push-pull (HIGH = deseleccionado).
     *    Clock de GPIOA ya habilitado por MX_GPIO_Init / SPI MspInit. */
    gpio_cs.Pin   = SD_CS_PIN;
    gpio_cs.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_cs.Pull  = GPIO_NOPULL;
    gpio_cs.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SD_CS_PORT, &gpio_cs);
    SD_CS_HIGH();

    /* 2) Inicializar buffer de 0xFF para SPI RX (se usa en SD_SPI_RxMulti) */
    memset(sd_ff_buf, 0xFF, sizeof(sd_ff_buf));

    sd_type = SD_TYPE_NONE;
    Stat = STA_NOINIT;

    /* 3) Velocidad SPI lenta para inicialización (≤400 kHz) */
    SD_SPI_SetSlow();

    /* 4) Enviar ≥74 clocks con CS HIGH (secuencia de arranque SD) */
    SD_CS_HIGH();
    for (n = 0; n < 10; n++) SD_SPI_TxByte(0xFF);

    /* 5) CMD0: Entrar en modo SPI (respuesta esperada: 0x01 = idle) */
    if (SD_SendCmd(SD_CMD0, 0) != 0x01)
    {
        SD_Deselect();
        return Stat;
    }

    /* 6) CMD8: Detectar versión SD */
    if (SD_SendCmd(SD_CMD8, 0x1AA) == 0x01)
    {
        /* ---- Tarjeta SD v2 ---- */
        for (n = 0; n < 4; n++) ocr[n] = SD_SPI_RxByte();

        if (ocr[2] == 0x01 && ocr[3] == 0xAA)
        {
            /* ACMD41(HCS=1): pollear hasta que salga de idle (max 2s) */
            start = HAL_GetTick();
            while ((HAL_GetTick() - start) < 2000)
            {
                if (SD_SendCmd(SD_ACMD41, 0x40000000) == 0x00) break;
            }

            /* CMD58: leer OCR para determinar SDHC/SDXC */
            if (SD_SendCmd(SD_CMD58, 0) == 0x00)
            {
                for (n = 0; n < 4; n++) ocr[n] = SD_SPI_RxByte();
                sd_type = (ocr[0] & 0x40) ? SD_TYPE_SDHC : SD_TYPE_SD2;
            }
        }
    }
    else
    {
        /* ---- Tarjeta SD v1 o MMC ---- */
        if (SD_SendCmd(SD_ACMD41, 0) <= 1)
        {
            sd_type  = SD_TYPE_SD1;
            cmd_poll = SD_ACMD41;
        }
        else
        {
            sd_type  = SD_TYPE_MMC;
            cmd_poll = SD_CMD1;
        }

        /* Pollear hasta que salga de idle (max 2s) */
        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 2000)
        {
            if (SD_SendCmd(cmd_poll, 0) == 0x00) break;
        }

        /* CMD16: Fijar tamaño de bloque 512 bytes (requerido para non-SDHC) */
        if (SD_SendCmd(SD_CMD16, 512) != 0x00)
            sd_type = SD_TYPE_NONE;
    }

    SD_Deselect();

    /* Si init exitoso → marcar disco listo y subir velocidad SPI */
    if (sd_type != SD_TYPE_NONE)
    {
        Stat &= ~STA_NOINIT;
        SD_SPI_SetFast();
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    Stat = (sd_type == SD_TYPE_NONE) ? STA_NOINIT : 0;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    DRESULT res = RES_ERROR;

    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    /* Para tarjetas non-SDHC: convertir sector LBA a dirección de byte */
    if (!(sd_type & SD_TYPE_SDHC)) sector *= 512;

    if (count == 1)
    {
        /* Lectura de un solo sector (CMD17) */
        if (SD_SendCmd(SD_CMD17, sector) == 0x00)
        {
            if (SD_RxDataBlock(buff, 512)) res = RES_OK;
        }
    }
    else
    {
        /* Lectura múltiple (CMD18 + CMD12 stop) */
        if (SD_SendCmd(SD_CMD18, sector) == 0x00)
        {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);
            SD_SendCmd(SD_CMD12, 0);
            if (count == 0) res = RES_OK;
        }
    }

    SD_Deselect();
    return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    DRESULT res = RES_ERROR;

    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    /* Para tarjetas non-SDHC: convertir sector LBA a dirección de byte */
    if (!(sd_type & SD_TYPE_SDHC)) sector *= 512;

    if (count == 1)
    {
        /* Escritura de un solo sector (CMD24) */
        if (SD_SendCmd(SD_CMD24, sector) == 0x00)
        {
            if (SD_TxDataBlock(buff, 0xFE)) res = RES_OK;
        }
    }
    else
    {
        /* Escritura múltiple (CMD25 + stop token 0xFD) */
        if (SD_SendCmd(SD_CMD25, sector) == 0x00)
        {
            do {
                if (!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            SD_TxDataBlock(NULL, 0xFD); /* Stop token */
            if (count == 0) res = RES_OK;
        }
    }

    SD_Deselect();
    return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    uint8_t csd[16];

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
    case CTRL_SYNC:
        /* Esperar a que la tarjeta termine operaciones pendientes */
        if (SD_Select()) res = RES_OK;
        SD_Deselect();
        break;

    case GET_SECTOR_COUNT:
        /* Leer CSD para calcular capacidad */
        if (SD_SendCmd(SD_CMD9, 0) == 0x00 && SD_RxDataBlock(csd, 16))
        {
            DWORD n_sectors;
            if ((csd[0] >> 6) == 1)
            {
                /* CSD v2 (SDHC/SDXC): capacidad = (C_SIZE+1) * 512KB */
                uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                                | ((uint32_t)csd[8] << 8) | csd[9];
                n_sectors = (c_size + 1) * 1024;
            }
            else
            {
                /* CSD v1: cálculo clásico con READ_BL_LEN y C_SIZE_MULT */
                uint8_t bl = (csd[5] & 0x0F);
                uint32_t cs = ((uint32_t)(csd[6] & 0x03) << 10)
                            | ((uint32_t)csd[7] << 2)
                            | ((csd[8] >> 6) & 0x03);
                uint8_t cm = ((csd[9] & 0x03) << 1)
                           | ((csd[10] >> 7) & 0x01);
                n_sectors = (cs + 1) << (bl + cm + 2 - 9);
            }
            *(DWORD *)buff = n_sectors;
            res = RES_OK;
        }
        SD_Deselect();
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1; /* Bloque de borrado = 1 sector */
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

