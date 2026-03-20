import sys
import serial
import struct
import time
import csv
import math
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel
from PyQt5.QtCore import QTimer

# ==========================================
# CONFIGURACIÓN DEL ROBOT Y PUERTO SERIE
# ==========================================
SERIAL_PORT = '/dev/ttyACM0'  # Cambia por tu puerto (ej. /dev/ttyACM0)
BAUD_RATE = 115200

# Ángulos de las ruedas respecto al eje X del robot (en grados). 
# ¡AJUSTA ESTOS VALORES SEGÚN TU PLANO PDF!
# Ejemplo común en SSL: [Front-Right, Back-Right, Back-Left, Front-Left]
WHEEL_ANGLES_DEG = [40, 140, 220, 320] 

# Parámetros del Encoder AS5600
ENCODER_RESOLUTION = 4096.0

# Configuración del gráfico
WINDOW_SIZE = 200  # Puntos en el historial del gráfico

class SSLRobotTelemetry(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Odometría y Telemetría - Robot SSL 40g")
        self.resize(1200, 700)
        
        # --- Variables de Estado ---
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = time.time()
        self.last_time = time.time()
        self.last_angles = [0, 0, 0, 0]
        self.velocities = [0.0, 0.0, 0.0, 0.0]
        self.raw_data = bytearray()
        
        # Historial para gráficos
        self.time_history = np.zeros(WINDOW_SIZE)
        self.vel_history = [np.zeros(WINDOW_SIZE) for _ in range(4)]
        
        # --- Inicializar Interfaz ---
        self.init_ui()
        
        # --- Inicializar Puerto Serie ---
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            print(f"Conectado a {SERIAL_PORT}")
        except Exception as e:
            print(f"Error abriendo el puerto serie: {e}")
            print("El programa iniciará en modo simulación/sin datos.")
            self.ser = None

        # --- Timer Principal (Bucle de actualización) ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(10) # 10 ms (100 Hz aprox)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- PANEL IZQUIERDO: Gráficos de Velocidad ---
        left_panel = QVBoxLayout()
        
        # Botón de Logging
        self.btn_log = QPushButton("Iniciar Grabación CSV")
        self.btn_log.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_log.clicked.connect(self.toggle_logging)
        left_panel.addWidget(self.btn_log)
        
        # Gráfico PyQtGraph
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plots = []
        self.curves = []
        colors = ['y', 'g', 'r', 'c']
        
        for i in range(4):
            p = self.plot_widget.addPlot(title=f"Velocidad Rueda {i+1} (rad/s)")
            p.showGrid(x=True, y=True)
            p.setYRange(-50, 50) # Rango de rad/s esperado (Ajustar si es necesario)
            curve = p.plot(pen=pg.mkPen(color=colors[i], width=2))
            self.plots.append(p)
            self.curves.append(curve)
            if i == 1: self.plot_widget.nextRow()
            
        left_panel.addWidget(self.plot_widget)
        layout.addLayout(left_panel, stretch=2)

        # --- PANEL DERECHO: Visualización 2D de la Base ---
        right_panel = QVBoxLayout()
        self.base_view = pg.PlotWidget(title="Contribución de Velocidad (Vista Superior)")
        self.base_view.setAspectLocked(True)
        self.base_view.setXRange(-2, 2)
        self.base_view.setYRange(-2, 2)
        self.base_view.hideAxis('bottom')
        self.base_view.hideAxis('left')
        
        # Dibujar chasis (círculo)
        theta = np.linspace(0, 2*np.pi, 100)
        self.base_view.plot(np.cos(theta), np.sin(theta), pen='w')
        
        # Dibujar ruedas (vectores/líneas)
        self.wheel_lines = []
        for angle_deg in WHEEL_ANGLES_DEG:
            angle_rad = math.radians(angle_deg)
            # Posición de la rueda en el borde del chasis
            x = math.cos(angle_rad)
            y = math.sin(angle_rad)
            # Línea inicial (largo cero)
            line = self.base_view.plot([x, x], [y, y], pen=pg.mkPen(width=5))
            self.wheel_lines.append((line, x, y, angle_rad))
            
        right_panel.addWidget(self.base_view)
        layout.addLayout(right_panel, stretch=1)

    def toggle_logging(self):
        if not self.is_logging:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"ssl_telemetry_{timestamp}.csv"
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Time(s)', 'Raw_Ang1', 'Raw_Ang2', 'Raw_Ang3', 'Raw_Ang4', 'Vel1(rad/s)', 'Vel2(rad/s)', 'Vel3(rad/s)', 'Vel4(rad/s)'])
            
            self.btn_log.setText(f"Detener Grabación ({filename})")
            self.btn_log.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 10px;")
            self.is_logging = True
            print(f"Logging iniciado: {filename}")
        else:
            self.is_logging = False
            self.btn_log.setText("Iniciar Grabación CSV")
            self.btn_log.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
            if self.csv_file:
                self.csv_file.close()
            print("Logging detenido.")

    def calculate_checksum(self, payload_bytes):
        return sum(payload_bytes) & 0xFF

    def update_loop(self):
        if self.ser is None:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        
        if self.ser.in_waiting > 0:
            self.raw_data.extend(self.ser.read(self.ser.in_waiting))

        # Procesar paquete de 12 bytes
        while len(self.raw_data) >= 12:
            if self.raw_data[0] == 0xAA and self.raw_data[1] == 0xBB:
                packet = self.raw_data[2:12]
                payload = packet[0:8]
                received_chk = packet[8]
                footer = packet[9]

                if footer == 0x0A and self.calculate_checksum(payload) == received_chk:
                    angles = struct.unpack('<HHHH', payload)
                    self.process_kinematics(angles, current_time, dt)
                    self.last_time = current_time
                
                self.raw_data = self.raw_data[12:]
            else:
                self.raw_data.pop(0)

    def process_kinematics(self, current_angles, current_time, dt):
        # Calcular velocidad para cada rueda
        for i in range(4):
            # Delta de ángulo en ticks
            delta_ticks = current_angles[i] - self.last_angles[i]
            
            # Unwrap: Manejar el cruce por cero (0 a 4095 o 4095 a 0)
            if delta_ticks > (ENCODER_RESOLUTION / 2):
                delta_ticks -= ENCODER_RESOLUTION
            elif delta_ticks < -(ENCODER_RESOLUTION / 2):
                delta_ticks += ENCODER_RESOLUTION
                
            # Convertir a radianes por segundo
            delta_rad = delta_ticks * (2 * math.pi / ENCODER_RESOLUTION)
            if dt > 0:
                self.velocities[i] = delta_rad / dt
            
            self.last_angles[i] = current_angles[i]
            
            # Actualizar historial para gráficos
            self.vel_history[i][:-1] = self.vel_history[i][1:]
            self.vel_history[i][-1] = self.velocities[i]

        self.time_history[:-1] = self.time_history[1:]
        self.time_history[-1] = current_time - self.start_time

        # Actualizar curvas de PyQtGraph
        for i in range(4):
            self.curves[i].setData(self.time_history, self.vel_history[i])

        # Actualizar visualización 2D de la base
        for i, (line, x, y, angle_rad) in enumerate(self.wheel_lines):
            # Escalar la velocidad para la visualización visual (multiplicador arbitrario)
            vel_scaled = self.velocities[i] * 0.05 
            
            # El vector de velocidad es tangente al círculo si las ruedas apuntan hacia adelante
            # Dependiendo del diseño SSL, la fuerza suele ser en (cos(alpha + 90), sin(alpha + 90))
            force_dir_x = math.cos(angle_rad + math.pi/2)
            force_dir_y = math.sin(angle_rad + math.pi/2)
            
            end_x = x + force_dir_x * vel_scaled
            end_y = y + force_dir_y * vel_scaled
            
            # Cambiar color según dirección
            color = 'g' if self.velocities[i] >= 0 else 'r'
            line.setData([x, end_x], [y, end_y], pen=pg.mkPen(color=color, width=5))

        # Guardar en CSV si está activo
        if self.is_logging and self.csv_writer:
            self.csv_writer.writerow([
                f"{current_time - self.start_time:.3f}",
                current_angles[0], current_angles[1], current_angles[2], current_angles[3],
                f"{self.velocities[0]:.2f}", f"{self.velocities[1]:.2f}", 
                f"{self.velocities[2]:.2f}", f"{self.velocities[3]:.2f}"
            ])

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = SSLRobotTelemetry()
    window.show()
    sys.exit(app.exec_())