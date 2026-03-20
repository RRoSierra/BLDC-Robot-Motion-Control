import sys
import serial
import struct
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

# --- CONFIGURACIÓN DEL PUERTO SERIE ---
# Cambia 'COM3' por el puerto de tu STM32 (ej. '/dev/ttyACM0' en Linux)
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 460800

# Parámetros del gráfico
WINDOW_SIZE = 500  # Cuántos puntos mostrar en la pantalla

class RealTimePlotter:
    def __init__(self):
        # Inicializar puerto serie
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        except Exception as e:
            print(f"Error abriendo puerto serie: {e}")
            sys.exit(1)

        # Buffer de datos para los 4 sensores
        self.data1 = np.zeros(WINDOW_SIZE)
        self.data2 = np.zeros(WINDOW_SIZE)
        self.data3 = np.zeros(WINDOW_SIZE)
        self.data4 = np.zeros(WINDOW_SIZE)
        self.ptr = 0

        # Configuración de la interfaz gráfica (PyQtGraph)
        self.app = QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="STM32 + AS5600 DMA Plotter")
        self.win.resize(1000, 600)
        self.win.setWindowTitle('Lectura de Ángulos en Tiempo Real')

        # Crear 4 sub-gráficos
        self.p1 = self.win.addPlot(title="Encoder 1")
        self.p2 = self.win.addPlot(title="Encoder 2")
        self.win.nextRow()
        self.p3 = self.win.addPlot(title="Encoder 3")
        self.p4 = self.win.addPlot(title="Encoder 4")

        # Configurar límites del eje Y (AS5600 va de 0 a 4095)
        for p in [self.p1, self.p2, self.p3, self.p4]:
            p.setYRange(0, 4095, padding=0)
            p.showGrid(x=True, y=True)

        # Crear las curvas
        self.curve1 = self.p1.plot(pen='y')
        self.curve2 = self.p2.plot(pen='g')
        self.curve3 = self.p3.plot(pen='r')
        self.curve4 = self.p4.plot(pen='c')

        # Buffer temporal para leer bytes
        self.raw_data = bytearray()

        # Configurar un Timer para actualizar los datos cada 10ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

    def calculate_checksum(self, payload_bytes):
        """Calcula un checksum simple (suma de bytes) igual al del STM32"""
        return sum(payload_bytes) & 0xFF

    def update(self):
        # Leer todo lo disponible en el buffer serie
        if self.ser.in_waiting > 0:
            self.raw_data.extend(self.ser.read(self.ser.in_waiting))

        # Buscar la trama en el buffer. 
        # Estructura esperada desde STM32 (12 bytes en total): 
        # [0xAA] [0xBB] [Ang1_H] [Ang1_L] [Ang2_H] [Ang2_L] [Ang3_H] [Ang3_L] [Ang4_H] [Ang4_L] [Checksum] [0x0A]
        
        while len(self.raw_data) >= 12:
            # Buscar el Header (0xAA 0xBB)
            if self.raw_data[0] == 0xAA and self.raw_data[1] == 0xBB:
                
                # Extraer el paquete (Payload + Checksum + Footer)
                packet = self.raw_data[2:12]
                payload = packet[0:8]
                received_checksum = packet[8]
                footer = packet[9]

                # Verificar footer y checksum
                if footer == 0x0A and self.calculate_checksum(payload) == received_checksum:
                    # Desempaquetar los 4 enteros de 16-bits (Big Endian o Little Endian según tu STM32)
                    # '<' significa Little Endian (estándar en ARM Cortex-M), 'H' es Unsigned Short (16 bits)
                    angles = struct.unpack('<HHHH', payload)
                    
                    self.update_data(angles[0], angles[1], angles[2], angles[3])
                
                # Eliminar la trama procesada (válida o corrupta) del buffer
                self.raw_data = self.raw_data[12:]
            else:
                # Si no encontramos el header, descartar el primer byte y seguir buscando
                self.raw_data.pop(0)

    def update_data(self, a1, a2, a3, a4):
        # Desplazar datos antiguos y añadir nuevos
        self.data1[:-1] = self.data1[1:]
        self.data1[-1] = a1
        
        self.data2[:-1] = self.data2[1:]
        self.data2[-1] = a2
        
        self.data3[:-1] = self.data3[1:]
        self.data3[-1] = a3
        
        self.data4[:-1] = self.data4[1:]
        self.data4[-1] = a4
        
        self.ptr += 1
        
        # Actualizar las gráficas
        self.curve1.setData(self.data1)
        self.curve2.setData(self.data2)
        self.curve3.setData(self.data3)
        self.curve4.setData(self.data4)

if __name__ == '__main__':
    plotter = RealTimePlotter()
    sys.exit(plotter.app.exec_())