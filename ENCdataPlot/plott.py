import sys
import serial
import struct
import time
import csv
import math
import os
import numpy as np
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer

# ==========================================
# CONFIGURACIÓN DEL ROBOT Y PUERTO SERIE
# ==========================================
SERIAL_PORT = '/dev/ttyACM0'  # Cambia por tu puerto real
BAUD_RATE = 115200

# Ángulos de las ruedas respecto al eje X del robot (en grados).
WHEEL_ANGLES_DEG = [40, 140, 220, 320] 

# Parámetros del Encoder AS5600
ENCODER_RESOLUTION = 4096.0

# Configuración del gráfico
WINDOW_SIZE = 200

# Paleta de Colores Sysmic
COLOR_BG = "#12141C"
COLOR_PANEL = "#1E222E"
COLOR_SYSMIC_BLUE = "#00A8E8"
COLOR_TEXT = "#E2E8F0"
COLOR_ACCENT = "#33C2FF"

class SSLRobotTelemetry(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sysmic Robotics - HMI Odometría SSL")
        self.resize(1280, 800)
        
        # --- Variables de Estado ---
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = time.time()
        self.last_time = time.time()
        self.last_angles = [0, 0, 0, 0]
        self.velocities = [0.0, 0.0, 0.0, 0.0]
        self.raw_data = bytearray()
        
        # Variables de Simulación
        self.simulation_mode = False
        self.sim_angles_float = [0.0, 0.0, 0.0, 0.0]
        
        # Historial para gráficos
        self.time_history = np.zeros(WINDOW_SIZE)
        self.vel_history = [np.zeros(WINDOW_SIZE) for _ in range(4)]
        
        # Configurar colores globales de PyQtGraph
        pg.setConfigOption('background', COLOR_BG)
        pg.setConfigOption('foreground', COLOR_TEXT)
        pg.setConfigOptions(antialias=True)
        
        # --- Inicializar Interfaz ---
        self.init_ui()
        
        # --- Inicializar Puerto Serie ---
        self.ser = None
        self.simulation_mode = True
        self.refresh_ports() # Llenar la lista de puertos inicial
        
        # --- Timer Principal ---
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(10) # 10 ms (100 Hz aprox)

    def init_ui(self):
        # Estilos QSS (Tema oscuro Sysmic)
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {COLOR_BG}; }}
            QLabel {{ color: {COLOR_TEXT}; font-family: Arial; }}
            QPushButton {{
                background-color: {COLOR_SYSMIC_BLUE}; color: white;
                border: none; border-radius: 8px; padding: 10px 15px;
                font-size: 14px; font-weight: bold;
            }}
            QPushButton:hover {{ background-color: {COLOR_ACCENT}; }}
            QPushButton:pressed {{ background-color: #007BA8; }}
            QComboBox {{
                background-color: {COLOR_PANEL}; color: {COLOR_TEXT};
                border: 1px solid {COLOR_SYSMIC_BLUE}; border-radius: 8px;
                padding: 8px; font-size: 14px;
            }}
            QComboBox::drop-down {{ border: none; }}
            QComboBox QAbstractItemView {{
                background-color: {COLOR_PANEL}; color: {COLOR_TEXT};
                selection-background-color: {COLOR_SYSMIC_BLUE};
            }}
            QFrame#HeaderFrame {{
                background-color: #080A0F; /* Contraste oscuro para la cabecera */
                border-bottom: 2px solid {COLOR_SYSMIC_BLUE};
            }}
            QFrame#ControlPanel {{
                background-color: {COLOR_PANEL};
                border-radius: 12px;
            }}
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # Layout principal sin márgenes para que el header cubra los bordes
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(15)

        # --- CABECERA (Oscura) ---
        self.header_frame = QtWidgets.QFrame()
        self.header_frame.setObjectName("HeaderFrame")
        header_layout = QHBoxLayout(self.header_frame)
        header_layout.setContentsMargins(20, 15, 20, 15)
        
        title = QLabel("HMI Robot Velocity Wheel Contribution")
        title.setStyleSheet(f"color: {COLOR_SYSMIC_BLUE}; font-size: 28px; font-weight: bold; letter-spacing: 2px;")
        header_layout.addWidget(title)
        header_layout.addStretch()
        
        # Cargar Logo Sysmic
        self.logo_label = QLabel()
        logo_path = "logo.png" # Nombre del archivo subido
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path).scaledToHeight(70, QtCore.Qt.SmoothTransformation)
            self.logo_label.setPixmap(pixmap)
        else:
            self.logo_label.setText("[ LOGO SYSMIC ]")
        header_layout.addWidget(self.logo_label)
        main_layout.addWidget(self.header_frame)

        # --- CONTENEDOR DEL CUERPO ---
        body_layout = QVBoxLayout()
        body_layout.setContentsMargins(20, 0, 20, 20)
        body_layout.setSpacing(15)
        main_layout.addLayout(body_layout)

        # --- PANEL DE CONTROL COM (Cuadro Redondeado) ---
        control_frame = QtWidgets.QFrame()
        control_frame.setObjectName("ControlPanel")
        control_layout = QHBoxLayout(control_frame)
        control_layout.setContentsMargins(15, 10, 15, 10)
        
        lbl_com = QLabel("Puerto COM:")
        lbl_com.setStyleSheet("font-weight: bold;")
        control_layout.addWidget(lbl_com)
        
        self.combo_ports = QtWidgets.QComboBox()
        self.combo_ports.setMinimumWidth(250)
        control_layout.addWidget(self.combo_ports)
        
        self.btn_refresh = QPushButton("↻ Refrescar")
        self.btn_refresh.setStyleSheet(f"background-color: transparent; border: 1px solid {COLOR_SYSMIC_BLUE}; color: {COLOR_SYSMIC_BLUE};")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        control_layout.addWidget(self.btn_refresh)
        
        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.clicked.connect(self.toggle_connection)
        control_layout.addWidget(self.btn_connect)
        
        self.lbl_status = QLabel("Estado: Simulación")
        self.lbl_status.setStyleSheet("color: #FFB000; font-weight: bold; margin-left: 20px;")
        control_layout.addWidget(self.lbl_status)
        
        control_layout.addStretch()
        body_layout.addWidget(control_frame)

        # --- ÁREA CENTRAL ---
        content_layout = QHBoxLayout()
        body_layout.addLayout(content_layout)

        # --- PANEL IZQUIERDO: Gráficos ---
        left_panel = QVBoxLayout()
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plots = []
        self.curves = []
        
        # Colores distintivos pero acordes al tema
        line_colors = ['#00A8E8', '#00FFCC', '#FFB000', '#FF3366']
        
        for i in range(4):
            p = self.plot_widget.addPlot(title=f"Velocidad Rueda {i+1} (rad/s)")
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setYRange(-30, 30) 
            
            # Estilizar ejes
            p.getAxis('left').setPen(COLOR_TEXT)
            p.getAxis('bottom').setPen(COLOR_TEXT)
            
            curve = p.plot(pen=pg.mkPen(color=line_colors[i], width=2.5))
            self.plots.append(p)
            self.curves.append(curve)
            if i == 1: self.plot_widget.nextRow()
            
        left_panel.addWidget(self.plot_widget)
        
        # Botón de Grabación
        self.btn_log = QPushButton("⭳ Iniciar Grabación CSV")
        self.btn_log.clicked.connect(self.toggle_logging)
        left_panel.addWidget(self.btn_log)
        
        content_layout.addLayout(left_panel, stretch=2)

        # --- PANEL DERECHO: Visualización 2D ---
        right_panel = QVBoxLayout()
        self.base_view = pg.PlotWidget(title="Base Kinematics (Top View)")
        self.base_view.setAspectLocked(True)
        self.base_view.setXRange(-1.5, 1.5)
        self.base_view.setYRange(-1.5, 1.5)
        self.base_view.hideAxis('bottom')
        self.base_view.hideAxis('left')
        
        self.draw_robot_base()
            
        right_panel.addWidget(self.base_view)
        content_layout.addLayout(right_panel, stretch=1)

    def draw_robot_base(self):
        """ Dibuja la silueta estética del robot SSL y sus ruedas """
        # 1. Base Circular con recorte (pateador) en la parte inferior
        cut_y = -0.75 # Altura del recorte horizontal (hacia abajo)
        start_angle = math.degrees(math.asin(cut_y)) # Esquina inferior derecha
        end_angle = 180 - start_angle                # Esquina inferior izquierda
        
        points = []
        # Generar puntos del arco desde la derecha, pasando por arriba, hasta la izquierda
        # El polígono se cerrará automáticamente con una línea recta en la parte inferior (el recorte)
        for a in np.linspace(start_angle, end_angle, 60):
            points.append(QtCore.QPointF(math.cos(math.radians(a))*1.0, math.sin(math.radians(a))*1.0))
            
        polygon = QtWidgets.QGraphicsPolygonItem(QtGui.QPolygonF(points))
        polygon.setBrush(pg.mkBrush(COLOR_PANEL))
        polygon.setPen(pg.mkPen('#404040', width=4))
        self.base_view.addItem(polygon)

        # 2. Círculos de Identificación (Patrón SSL de la tapa superior)
        def add_pattern_circle(cx, cy, color, radius=0.18):
            circle = QtWidgets.QGraphicsEllipseItem(cx - radius, cy - radius, radius * 2, radius * 2)
            circle.setBrush(pg.mkBrush(color))
            circle.setPen(pg.mkPen('#000000', width=2))
            self.base_view.addItem(circle)

        add_pattern_circle(0, 0, '#FFD700', radius=0.25)     # Centro (Amarillo)
        add_pattern_circle(-0.45, 0.45, '#FF00FF')           # Front-Left (Magenta)
        add_pattern_circle(0.45, 0.45, '#00FF00')            # Front-Right (Verde)
        add_pattern_circle(-0.45, -0.45, COLOR_SYSMIC_BLUE)  # Back-Left (Azul Sysmic)
        add_pattern_circle(0.45, -0.45, '#FF8C00')           # Back-Right (Naranja)

        # 3. Ruedas (Huecos y Llantas) y Vectores
        self.wheel_lines = []
        for angle_deg in WHEEL_ANGLES_DEG:
            angle_rad = math.radians(angle_deg)
            x = math.cos(angle_rad) * 1.0 # Ruedas alejadas al radio de valor 1.0
            y = math.sin(angle_rad) * 1.0
            
            # Dibujar Rueda
            wheel = QtWidgets.QGraphicsRectItem(-0.25, -0.08, 0.5, 0.16)
            wheel.setPos(x, y)
            wheel.setRotation(angle_deg + 90) # Tangente al radio
            wheel.setBrush(pg.mkBrush('#111111')) # Llanta oscura
            wheel.setPen(pg.mkPen(COLOR_SYSMIC_BLUE, width=2))
            self.base_view.addItem(wheel)

            # Línea para el Vector de Velocidad
        line = self.base_view.plot([x, x], [y, y], pen=pg.mkPen(width=5))
        self.wheel_lines.append((line, x, y, angle_rad))

    def refresh_ports(self):
        """ Escanea y actualiza la lista de puertos serie disponibles """
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.combo_ports.addItem(port.device)
        if not ports:
            self.combo_ports.addItem("Ningún puerto encontrado")

    def toggle_connection(self):
        """ Conecta o desconecta el puerto serial seleccionado """
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.simulation_mode = True
            self.btn_connect.setText("Conectar")
            self.btn_connect.setStyleSheet(f"background-color: {COLOR_SYSMIC_BLUE}; color: white;")
            self.lbl_status.setText("Estado: Simulación")
            self.lbl_status.setStyleSheet("color: #FFB000; font-weight: bold; margin-left: 20px;")
            self.combo_ports.setEnabled(True)
            self.btn_refresh.setEnabled(True)
        else:
            port = self.combo_ports.currentText()
            if port and "Ningún" not in port:
                try:
                    self.ser = serial.Serial(port, BAUD_RATE, timeout=0.01)
                    self.simulation_mode = False
                    self.btn_connect.setText("Desconectar")
                    self.btn_connect.setStyleSheet("background-color: #EF4444; color: white;") # Rojo suave
                    self.lbl_status.setText(f"Estado: Conectado ({port})")
                    self.lbl_status.setStyleSheet("color: #10B981; font-weight: bold; margin-left: 20px;") # Verde
                    self.combo_ports.setEnabled(False)
                    self.btn_refresh.setEnabled(False)
                except Exception as e:
                    print(f"Error conectando: {e}")
                    self.lbl_status.setText("Error de conexión")
                    self.lbl_status.setStyleSheet("color: #EF4444; font-weight: bold; margin-left: 20px;")

    def toggle_logging(self):
        if not self.is_logging:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"sysmic_telemetry_{timestamp}.csv"
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Time(s)', 'Raw1', 'Raw2', 'Raw3', 'Raw4', 'Vel1', 'Vel2', 'Vel3', 'Vel4'])
            
            self.btn_log.setText(f"⏹ Detener Grabación ({filename})")
            self.btn_log.setStyleSheet("background-color: #EF4444; color: white;") # Rojo suave
            self.is_logging = True
        else:
            self.is_logging = False
            self.btn_log.setText("⭳ Iniciar Grabación CSV")
            self.btn_log.setStyleSheet(f"background-color: {COLOR_SYSMIC_BLUE}; color: white;")
            if self.csv_file:
                self.csv_file.close()

    def calculate_checksum(self, payload_bytes):
        return sum(payload_bytes) & 0xFF

    def update_loop(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Prevenir dt = 0 que causa división por cero
        if dt < 0.001: 
            return

        if self.simulation_mode:
            self.run_simulation(current_time, dt)
            return

        # Modo Serie Real
        if self.ser and self.ser.in_waiting > 0:
            self.raw_data.extend(self.ser.read(self.ser.in_waiting))

        latest_angles = None
        
        # Procesar buffer (deslizando inteligentemente para no perder paquetes)
        while len(self.raw_data) >= 12:
            if self.raw_data[0] == 0xAA and self.raw_data[1] == 0xBB:
                if self.raw_data[11] == 0x0A: # Verificar Footer
                    payload = self.raw_data[2:10]
                    chk = self.raw_data[10]
                    
                    if self.calculate_checksum(payload) == chk:
                        # Guardamos solo el último paquete válido del buffer para evitar lag visual
                        latest_angles = struct.unpack('<HHHH', payload)
                        
                    self.raw_data = self.raw_data[12:] # Avanzar 1 trama completa
                else:
                    self.raw_data.pop(0) # Falso positivo de cabecera, avanzar 1 byte
            else:
                self.raw_data.pop(0) # Buscar cabecera

        # Procesar matemáticas solo con el paquete más nuevo
        if latest_angles is not None:
            self.process_kinematics(latest_angles, current_time, dt)
            self.last_time = current_time

    def run_simulation(self, current_time, dt):
        """ Genera datos falsos para probar la HMI visualmente sin robot """
        sim_speeds = [
            math.sin(current_time * 2.0) * 20,       # Oscila adelante/atrás
            math.cos(current_time * 1.5) * 15,       
            math.sin(current_time * 1.8) * -20,      
            math.cos(current_time * 2.2) * 25        
        ]
        
        current_angles = []
        for i in range(4):
            # Integrar velocidad para simular la posición del encoder
            ticks_per_sec = sim_speeds[i] * (ENCODER_RESOLUTION / (2 * math.pi))
            self.sim_angles_float[i] += ticks_per_sec * dt
            self.sim_angles_float[i] %= ENCODER_RESOLUTION
            current_angles.append(int(self.sim_angles_float[i]))
            
        self.process_kinematics(current_angles, current_time, dt)
        self.last_time = current_time

    def process_kinematics(self, current_angles, current_time, dt):
        for i in range(4):
            delta_ticks = current_angles[i] - self.last_angles[i]
            
            # Unwrap: Manejar el cruce por cero del encoder absoluto
            if delta_ticks > (ENCODER_RESOLUTION / 2):
                delta_ticks -= ENCODER_RESOLUTION
            elif delta_ticks < -(ENCODER_RESOLUTION / 2):
                delta_ticks += ENCODER_RESOLUTION
                
            delta_rad = delta_ticks * (2 * math.pi / ENCODER_RESOLUTION)
            self.velocities[i] = delta_rad / dt
            self.last_angles[i] = current_angles[i]
            
            # Historial para gráficos
            self.vel_history[i][:-1] = self.vel_history[i][1:]
            self.vel_history[i][-1] = self.velocities[i]

        self.time_history[:-1] = self.time_history[1:]
        self.time_history[-1] = current_time - self.start_time

        # 1. Actualizar Gráficos PyqtGraph
        for i in range(4):
            self.curves[i].setData(self.time_history, self.vel_history[i])

        # 2. Actualizar Vectores de la Vista 2D
        for i, (line, x, y, angle_rad) in enumerate(self.wheel_lines):
            vel_scaled = self.velocities[i] * 0.02 # Factor de escala visual para la flecha
            
            # Dirección de empuje de la rueda (perpendicular al radio para base omni)
            force_dir_x = math.cos(angle_rad + math.pi/2)
            force_dir_y = math.sin(angle_rad + math.pi/2)
            
            end_x = x + force_dir_x * vel_scaled
            end_y = y + force_dir_y * vel_scaled
            
            # Color verde para empuje positivo, Magenta para negativo
            vec_color = '#00FF00' if self.velocities[i] >= 0 else '#FF00FF'
            line.setData([x, end_x], [y, end_y], pen=pg.mkPen(color=vec_color, width=5))

        # 3. Grabar CSV
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