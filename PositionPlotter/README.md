# PositionPlotter — Firmware de Telemetría de Encoders AS5600

## Descripción general

Este firmware corre en una **STM32F767ZI** (placa Nucleo-144). Su única tarea es leer en tiempo real los ángulos crudos de **4 encoders magnéticos AS5600** (uno por bus I2C), empaquetar los datos en una trama binaria compacta y enviarla por **USART3 a 115 200 baud** hacia un agente externo (PC, Raspberry Pi, etc.) que los analiza y grafica.

---

## Hardware

| Elemento | Detalle |
|---|---|
| MCU | STM32F767ZI — Nucleo-144 |
| Encoders | 4 × AS5600 (magnético, 12 bits, rango 0–4095) |
| Buses I2C | I2C1, I2C2, I2C3, I2C4 (transferencias por DMA) |
| Puerto serie | USART3 — PD8 (RX) / PD9 (TX) |
| Interfaz PC | ST-Link Virtual COM Port → `ttyACM0` (Linux) o `COMx` (Windows) |

---

## Flujo de funcionamiento

```
[ARRANQUE]
    │
    ├─► Inicializa periféricos: GPIO, DMA, I2C×4, USART3, TIM1
    │
    ├─► AS5600_Reset_Calibration() × 4
    │     Limpia ZPOS, MPOS, MANG en RAM del sensor
    │     → rango completo garantizado [0 ... 4095]
    │
    ├─► AS5600_Diagnostic()
    │     Envía por UART3 diagnóstico ASCII de los 4 sensores:
    │     conexión, ZPOS, MPOS, MANG, STATUS, AGC, MAGNITUD
    │
    └─► HAL_TIM_Base_Start_IT(&htim1)  ← Arranca el ciclo de 100 Hz

[CICLO PRINCIPAL — 100 Hz / 10 ms]
    │
    ├─► TIM1 Update IRQ (cada 10 ms)
    │     Lanza las 4 lecturas DMA en paralelo:
    │       I2C1 → as5600_rx_buf[0][2 bytes]
    │       I2C2 → as5600_rx_buf[1][2 bytes]
    │       I2C3 → as5600_rx_buf[2][2 bytes]
    │       I2C4 → as5600_rx_buf[3][2 bytes]
    │
    ├─► HAL_I2C_MemRxCpltCallback() (dispara 4 veces, una por bus)
    │     Acumula flags: i2c_cplt_flags |= (1 << bus_index)
    │     Cuando los 4 flags están en 1 (0x0F):
    │       ・Ensambla uart_tx_data.angles[0..3] (Big→Little Endian)
    │       ・Calcula checksum LSB del payload (8 bytes de ángulos)
    │       ・Pone data_ready = 1
    │
    └─► Loop principal (while 1)
          Si data_ready == 1:
            data_ready = 0
            HAL_UART_Transmit(&huart3, trama, 12 bytes, timeout 5 ms)
```

> **Manejo de fallos:** Si un sensor no contesta (NACK), `HAL_I2C_ErrorCallback` fuerza su bit en `i2c_cplt_flags` para no bloquear el ciclo. El ángulo de ese sensor quedará con el valor de la última lectura válida.

---

## Configuración del timer (cadencia de muestreo)

El timer se reconfigura en `main.c` en tiempo de ejecución:

```c
htim1.Instance->PSC = 9600 - 1;   // Prescaler → reloj de 10 kHz
htim1.Instance->ARR = 100  - 1;   // Period    → 10 ms
```

Con SYSCLK = 96 MHz:  
`f_TIM1 = 96 000 000 / 9600 / 100 = 100 Hz`

**El MCU transmite exactamente 1 trama cada 10 ms → 100 tramas por segundo.**

---

## Puerto serie — Parámetros de conexión

| Parámetro | Valor |
|---|---|
| Baud rate | **115 200** |
| Bits de datos | 8 |
| Paridad | Ninguna |
| Bits de parada | 1 |
| Control de flujo | Ninguno |
| Modo UART | TX + RX (el MCU solo transmite; no espera comandos) |

---

## Protocolo de trama binaria

### Estructura `DataFrame_t` (12 bytes, sin padding)

```c
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2];   // [0]=0xAA  [1]=0xBB
    uint16_t angles[4];   // 4 ángulos en Little Endian (uint16)
    uint8_t  checksum;    // LSB de la suma de los 8 bytes de payload
    uint8_t  footer;      // 0x0A
} DataFrame_t;
#pragma pack(pop)
```

### Mapa de bytes

| Offset | Tamaño | Contenido | Valor fijo / Rango |
|--------|--------|-----------|--------------------|
| 0 | 1 byte | Header[0] | `0xAA` |
| 1 | 1 byte | Header[1] | `0xBB` |
| 2–3 | uint16 LE | Ángulo sensor 1 (I2C1) | 0 … 4095 |
| 4–5 | uint16 LE | Ángulo sensor 2 (I2C2) | 0 … 4095 |
| 6–7 | uint16 LE | Ángulo sensor 3 (I2C3) | 0 … 4095 |
| 8–9 | uint16 LE | Ángulo sensor 4 (I2C4) | 0 … 4095 |
| 10 | 1 byte | Checksum | LSB(Σ bytes 2..9) |
| 11 | 1 byte | Footer | `0x0A` |

**Total: 12 bytes por trama.**

### Cálculo del checksum

```
checksum = (byte[2] + byte[3] + byte[4] + byte[5] +
            byte[6] + byte[7] + byte[8] + byte[9]) & 0xFF
```

Es la suma aritmética de los 8 bytes de payload (los 4 ángulos en Little Endian), truncada al byte bajo.

### Ejemplo de trama válida

Sensores leyendo: S1=1000, S2=2048, S3=3000, S4=500

```
Offset:  00   01   02   03   04   05   06   07   08   09   10   11
Valor:  [AA] [BB] [E8] [03] [00] [08] [B8] [0B] [F4] [01] [XX] [0A]
                  |     |   |     |   |     |   |     |
                  └─────┘   └─────┘   └─────┘   └─────┘
                  1000 LE   2048 LE   3000 LE    500 LE
```

Checksum: `(0xE8+0x03+0x00+0x08+0xB8+0x0B+0xF4+0x01) & 0xFF = 0xB7`

---

## Conversión de ángulo crudo a unidades físicas

El AS5600 tiene resolución de **12 bits → 4096 pasos por vuelta completa**.

```
Grados = raw * (360.0 / 4096)
Radianes = raw * (2π / 4096)
```

Ejemplos:
| raw | Grados | Radianes |
|-----|--------|----------|
| 0 | 0.00° | 0.000 |
| 1024 | 90.00° | π/2 |
| 2048 | 180.00° | π |
| 4095 | 359.91° | ~2π |

---

## Lo que el agente externo DEBE hacer

### 1. Abrir el puerto serie
```python
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
```

### 2. Sincronizar y parsear tramas

El buffer puede contener basura inicial (diagnóstico ASCII del arranque) o tramas cortadas. El algoritmo correcto es deslizante:

```python
import struct

raw_buffer = bytearray()

def calculate_checksum(payload_bytes):
    return sum(payload_bytes) & 0xFF

def read_frames(ser):
    """Retorna lista de tuplas (ang1, ang2, ang3, ang4) con tramas válidas."""
    frames = []

    # 1. Leer todos los bytes disponibles y acumular
    if ser.in_waiting > 0:
        raw_buffer.extend(ser.read(ser.in_waiting))

    # 2. Procesar el buffer buscando tramas completas
    while len(raw_buffer) >= 12:
        idx0 = raw_buffer[0]
        idx1 = raw_buffer[1]

        if idx0 == 0xAA and idx1 == 0xBB:           # Cabecera encontrada
            if raw_buffer[11] == 0x0A:               # Footer correcto
                payload = raw_buffer[2:10]           # 8 bytes de ángulos
                chk     = raw_buffer[10]

                if calculate_checksum(payload) == chk:
                    angles = struct.unpack('<HHHH', payload)  # Little Endian
                    frames.append(angles)

                raw_buffer[:] = raw_buffer[12:]      # Avanzar trama completa
            else:
                raw_buffer.pop(0)                    # Falso positivo, buscar de nuevo
        else:
            raw_buffer.pop(0)                        # Sin cabecera, avanzar 1 byte

    return frames
```

### 3. Interpretar los valores

```python
ENCODER_RESOLUTION = 4096.0
import math

for angles in read_frames(ser):
    for i, raw in enumerate(angles):
        degrees = raw * 360.0 / ENCODER_RESOLUTION
        radians = raw * 2 * math.pi / ENCODER_RESOLUTION
        print(f"Sensor {i+1}: raw={raw:4d}  {degrees:7.2f}°  {radians:.4f} rad")
```

### 4. Calcular velocidad angular (diferenciación discreta)

```python
last_angles = [0, 0, 0, 0]

def compute_velocities(current_angles, dt):
    """Retorna velocidades en rad/s con unwrap del encoder absoluto."""
    velocities = []
    for i, curr in enumerate(current_angles):
        delta = curr - last_angles[i]

        # Unwrap: compensar cruce por cero (0 ↔ 4095)
        if delta >  ENCODER_RESOLUTION / 2:
            delta -= ENCODER_RESOLUTION
        elif delta < -ENCODER_RESOLUTION / 2:
            delta += ENCODER_RESOLUTION

        vel_rad_s = (delta * 2 * math.pi / ENCODER_RESOLUTION) / dt
        velocities.append(vel_rad_s)
        last_angles[i] = curr

    return velocities
```

---

## Lo que el agente externo NO necesita enviar

**El MCU no espera ningún comando.** El streaming comienza automáticamente al arrancar y corre en forma perpetua a 100 Hz.  
El pin RX (PD9) del USART3 está configurado pero el firmware no tiene ningún handler de recepción activo. Cualquier byte enviado al MCU es ignorado.

---

## Mensaje de diagnóstico al arrancar (ASCII)

Antes de iniciar el streaming binario, el MCU envía por UART3 un bloque de texto legible por humano para verificar la salud del hardware. Ejemplo:

```
========== AS5600 DIAGNOSTIC ==========

--- Sensor 1 (I2C1) ---
  Estado: CONECTADO
  ZPOS: 0  |  MPOS: 0  |  MANG: 0
  AGC: 83  |  Magnitud: 3104
  Iman: MD=1 ML=0 MH=0 -> OK - Intensidad aceptable

--- Sensor 2 (I2C2) ---
  Estado: CONECTADO
  ...

========================================
```

Un agente externo puede ignorar este bloque: el algoritmo de sincronización deslizante descartará automáticamente todos los bytes ASCII hasta encontrar la cabecera `0xAA 0xBB`.

---

## Resumen rápido para integración

```
Puerto :  115200 8N1 (no flow control)
Trama  :  12 bytes fijos
          [AA][BB][A1_L][A1_H][A2_L][A2_H][A3_L][A3_H][A4_L][A4_H][CHK][0A]
Cadencia: 100 Hz  (1 trama cada 10 ms)
Payload : 4 × uint16 Little Endian — rango 0..4095 (= 0°..359.91°)
Checksum: suma LSB de los 8 bytes de payload (offsets 2..9)
Sentido : MCU → PC  (unidireccional, el MCU no procesa RX)
```
