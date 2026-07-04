"""
Sistema de Control y Monitoreo de Dron - Python
================================================

Interfaz gráfica para controlar el dron ESP32 vía WiFi.

- Conexión WiFi al dron
- Envío de comandos (ARM, THROTTLE, PID)
- Visualización en tiempo real de telemetría
- Gráficos de roll, pitch, altura y potencia de motores
- Ajuste de parámetros PID

Requisitos:
pip install matplotlib numpy

Uso:
1. Conecta tu PC al WiFi "DroneControl" (contraseña: drone1234)
2. Ejecuta este script
3. Los datos se visualizarán en tiempo real
"""

import socket
import threading
import time
import sys
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button, Slider, TextBox
import numpy as np

# ============================================
# CONFIGURACIÓN DE CONEXIÓN
# ============================================
DRONE_IP = "192.168.4.1"  # IP del Access Point de la ESP32
DRONE_PORT = 8888
TIMEOUT = 5.0

# ============================================
# CLASE PRINCIPAL DEL CONTROLADOR
# ============================================
class DroneController:
    def _init_(self):
        """Inicializa el controlador del dron"""
        self.socket = None
        self.connected = False
        self.running = False
        
        # Variables de telemetría
        self.roll = 0.0
        self.pitch = 0.0
        self.throttle = 0
        self.altitude = 0.0
        self.motor_fl = 0
        self.motor_fr = 0
        self.motor_bl = 0
        self.motor_br = 0
        
        # Historial para gráficos (últimos 200 puntos)
        self.max_points = 200
        self.time_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.altitude_data = deque(maxlen=self.max_points)
        self.throttle_data = deque(maxlen=self.max_points)
        self.motor_fl_data = deque(maxlen=self.max_points)
        self.motor_fr_data = deque(maxlen=self.max_points)
        self.motor_bl_data = deque(maxlen=self.max_points)
        self.motor_br_data = deque(maxlen=self.max_points)
        
        self.start_time = time.time()
        
        # Estado del sistema
        self.armed = False
        self.max_throttle = 100
        
        # PID actuales
        self.pid_roll = {"kp": 1.0, "ki": 0.05, "kd": 0.5}
        self.pid_pitch = {"kp": 1.0, "ki": 0.05, "kd": 0.5}
        
        # Thread para recibir datos
        self.receive_thread = None
        
    def connect(self):
        """Conecta al dron vía WiFi"""
        try:
            print(f"Conectando a {DRONE_IP}:{DRONE_PORT}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(TIMEOUT)
            self.socket.connect((DRONE_IP, DRONE_PORT))
            self.connected = True
            self.running = True
            print("✓ Conexión establecida")
            
            # Iniciar thread de recepción
            self.receive_thread = threading.Thread(target=self._receive_data, daemon=True)
            self.receive_thread.start()
            
            return True
        except Exception as e:
            print(f"✗ Error de conexión: {e}")
            print("\nVerifica que:")
            print("1. Estés conectado al WiFi 'DroneControl'")
            print("2. La ESP32 esté encendida y funcionando")
            print("3. La IP sea correcta (normalmente 192.168.4.1)")
            self.connected = False
            return False
    
    def disconnect(self):
        """Desconecta del dron"""
        self.running = False
        if self.socket:
            try:
                self.send_command("DISARM")
                time.sleep(0.1)
                self.socket.close()
            except:
                pass
        self.connected = False
        print("Desconectado del dron")
    
    def send_command(self, command):
        """Envía un comando al dron"""
        if not self.connected:
            print("✗ No conectado al dron")
            return False
        
        try:
            message = command + "\n"
            self.socket.sendall(message.encode())
            print(f"→ Enviado: {command}")
            return True
        except Exception as e:
            print(f"✗ Error enviando comando: {e}")
            self.connected = False
            return False
    
    def _receive_data(self):
        """Thread que recibe datos de telemetría continuamente"""
        buffer = ""
        
        while self.running:
            try:
                data = self.socket.recv(1024).decode('utf-8')
                if not data:
                    print("✗ Conexión cerrada por el dron")
                    self.connected = False
                    break
                
                buffer += data
                
                # Procesar líneas completas
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line:
                        self._process_message(line)
                        
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"✗ Error recibiendo datos: {e}")
                    self.connected = False
                break
    
    def _process_message(self, message):
        """Procesa un mensaje recibido del dron"""
        if message.startswith("TELEMETRY:"):
            # Formato: TELEMETRY:roll,pitch,throttle,alt,FL,FR,BL,BR
            try:
                parts = message.split(':')[1].split(',')
                self.roll = float(parts[0])
                self.pitch = float(parts[1])
                self.throttle = int(parts[2])
                self.altitude = float(parts[3])
                self.motor_fl = int(parts[4])
                self.motor_fr = int(parts[5])
                self.motor_bl = int(parts[6])
                self.motor_br = int(parts[7])
                
                # Agregar a historial
                current_time = time.time() - self.start_time
                self.time_data.append(current_time)
                self.roll_data.append(self.roll)
                self.pitch_data.append(self.pitch)
                self.altitude_data.append(self.altitude)
                self.throttle_data.append(self.throttle)
                self.motor_fl_data.append(self.motor_fl)
                self.motor_fr_data.append(self.motor_fr)
                self.motor_bl_data.append(self.motor_bl)
                self.motor_br_data.append(self.motor_br)
                
            except Exception as e:
                print(f"Error procesando telemetría: {e}")
        
        elif message.startswith("OK:"):
            print(f"✓ {message}")
        
        elif message.startswith("ERROR:"):
            print(f"✗ {message}")
    
    # ============================================
    # COMANDOS DE CONTROL
    # ============================================
    
    def arm(self):
        """Armar el dron (habilitar motores)"""
        if self.send_command("ARM"):
            self.armed = True
            print("⚠ DRON ARMADO - Motores habilitados")
    
    def disarm(self):
        """Desarmar el dron (deshabilitar motores)"""
        if self.send_command("DISARM"):
            self.armed = False
            print("✓ Dron desarmado - Motores deshabilitados")
    
    def set_throttle(self, value):
        """Establecer potencia del dron (0-255)"""
        value = max(0, min(255, int(value)))
        self.send_command(f"THROTTLE:{value}")
    
    def set_max_throttle(self, value):
        """Establecer límite máximo de potencia (seguridad)"""
        value = max(0, min(255, int(value)))
        self.max_throttle = value
        self.send_command(f"MAXTHROTTLE:{value}")
    
    def set_pid_roll(self, kp, ki, kd):
        """Configurar PID del roll"""
        self.pid_roll = {"kp": kp, "ki": ki, "kd": kd}
        self.send_command(f"PID_ROLL:{kp},{ki},{kd}")
    
    def set_pid_pitch(self, kp, ki, kd):
        """Configurar PID del pitch"""
        self.pid_pitch = {"kp": kp, "ki": ki, "kd": kd}
        self.send_command(f"PID_PITCH:{kp},{ki},{kd}")
    
    def request_status(self):
        """Solicitar estado actual del dron"""
        self.send_command("STATUS")

# ============================================
# INTERFAZ GRÁFICA CON MATPLOTLIB
# ============================================
class DroneGUI:
    def _init_(self, controller):
        """Inicializa la interfaz gráfica"""
        self.controller = controller
        
        # Crear figura con múltiples subplots
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('Control de Dron ESP32')
        
        try:
            manager = plt.get_current_fig_manager()
            manager.window.state('zoomed')
        except:
            pass  # Si falla, continuar con el tamaño normal
        
        # Configurar grid
        gs = gridspec.GridSpec(4, 3, figure=self.fig, hspace=0.4, wspace=0.3)
        
        # Gráficos
        self.ax_orientation = self.fig.add_subplot(gs[0, :2])  # Roll y Pitch
        self.ax_altitude = self.fig.add_subplot(gs[1, :2])      # Altura
        self.ax_motors = self.fig.add_subplot(gs[2, :2])        # Motores
        self.ax_throttle = self.fig.add_subplot(gs[3, :2])      # Throttle
        
        # Panel de control (columna derecha)
        self.ax_controls = self.fig.add_subplot(gs[:, 2])
        self.ax_controls.axis('off')
        
        self._setup_plots()
        self._setup_controls()
        
        # Animación
        self.anim = FuncAnimation(self.fig, self._update_plots, 
                                 interval=50, blit=False)
    
    def _setup_plots(self):
        """Configura los gráficos"""
        # Roll y Pitch
        self.ax_orientation.set_title('Orientación (Roll y Pitch)', fontsize=12, fontweight='bold')
        self.ax_orientation.set_ylabel('Ángulo (°)')
        self.ax_orientation.set_xlabel('Tiempo (s)')
        self.ax_orientation.grid(True, alpha=0.3)
        self.ax_orientation.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.line_roll, = self.ax_orientation.plot([], [], 'b-', label='Roll', linewidth=2)
        self.line_pitch, = self.ax_orientation.plot([], [], 'r-', label='Pitch', linewidth=2)
        self.ax_orientation.legend(loc='upper right')
        self.ax_orientation.set_ylim(-45, 45)
        
        # Altura
        self.ax_altitude.set_title('Altura Estimada', fontsize=12, fontweight='bold')
        self.ax_altitude.set_ylabel('Altura (m)')
        self.ax_altitude.set_xlabel('Tiempo (s)')
        self.ax_altitude.grid(True, alpha=0.3)
        self.line_altitude, = self.ax_altitude.plot([], [], 'g-', linewidth=2)
        self.ax_altitude.set_ylim(0, 5)
        
        # Motores
        self.ax_motors.set_title('Potencia de Motores', fontsize=12, fontweight='bold')
        self.ax_motors.set_ylabel('PWM (0-255)')
        self.ax_motors.set_xlabel('Tiempo (s)')
        self.ax_motors.grid(True, alpha=0.3)
        self.line_motor_fl, = self.ax_motors.plot([], [], label='FL', linewidth=1.5)
        self.line_motor_fr, = self.ax_motors.plot([], [], label='FR', linewidth=1.5)
        self.line_motor_bl, = self.ax_motors.plot([], [], label='BL', linewidth=1.5)
        self.line_motor_br, = self.ax_motors.plot([], [], label='BR', linewidth=1.5)
        self.ax_motors.legend(loc='upper right')
        self.ax_motors.set_ylim(0, 255)
        
        # Throttle
        self.ax_throttle.set_title('Throttle (Potencia Base)', fontsize=12, fontweight='bold')
        self.ax_throttle.set_ylabel('PWM (0-255)')
        self.ax_throttle.set_xlabel('Tiempo (s)')
        self.ax_throttle.grid(True, alpha=0.3)
        self.line_throttle, = self.ax_throttle.plot([], [], 'm-', linewidth=2)
        self.ax_throttle.set_ylim(0, 255)
    
    def _setup_controls(self):
        """Configura el panel de control"""
        # Título
        self.ax_controls.text(0.5, 1.1, 'PANEL DE CONTROL', 
                             ha='center', va='top', fontsize=14, fontweight='bold')
        
        # Estado de conexión
        self.status_text = self.ax_controls.text(0.5, 1.05, '⚫ Desconectado', 
                                                ha='center', va='top', fontsize=10)
        
        # Botones ARM/DISARM
        ax_arm = plt.axes([0.70, 0.83, 0.12, 0.04])
        ax_disarm = plt.axes([0.83, 0.83, 0.12, 0.04])
        self.btn_arm = Button(ax_arm, 'ARM', color='lightgreen')
        self.btn_disarm = Button(ax_disarm, 'DISARM', color='lightcoral')
        self.btn_arm.on_clicked(lambda event: self.controller.arm())
        self.btn_disarm.on_clicked(lambda event: self.controller.disarm())
        
        # Slider de Throttle
        self.ax_controls.text(0.1, 0.87, 'Throttle:', fontsize=10, fontweight='bold')
        ax_throttle_slider = plt.axes([0.70, 0.74, 0.25, 0.02])
        self.slider_throttle = Slider(ax_throttle_slider, '', 0, 255, 
                                      valinit=0, valstep=5, color='purple')
        self.slider_throttle.on_changed(self._on_throttle_change)
        
        # Slider de Max Throttle
        self.ax_controls.text(0.1, 0.75, 'Max Throttle:', fontsize=10, fontweight='bold')
        ax_max_throttle = plt.axes([0.70, 0.65, 0.25, 0.02])
        self.slider_max_throttle = Slider(ax_max_throttle, '', 0, 255, 
                                          valinit=100, valstep=5, color='orange')
        self.slider_max_throttle.on_changed(self._on_max_throttle_change)
        
        # PID Roll
        self.ax_controls.text(0.5, 0.65, 'PID ROLL', 
                             ha='center', fontsize=11, fontweight='bold')
        
        ax_roll_kp = plt.axes([0.70, 0.55, 0.25, 0.02])
        ax_roll_ki = plt.axes([0.70, 0.50, 0.25, 0.02])
        ax_roll_kd = plt.axes([0.70, 0.45, 0.25, 0.02])
        
        self.ax_controls.text(0.2, 0.61, 'Kp:', ha='right', fontsize=9)
        self.ax_controls.text(0.198, 0.545, 'Ki:', ha='right', fontsize=9)
        self.ax_controls.text(0.2, 0.48, 'Kd:', ha='right', fontsize=9)
        
        self.slider_roll_kp = Slider(ax_roll_kp, '', 0, 5, valinit=1.0, valstep=0.1)
        self.slider_roll_ki = Slider(ax_roll_ki, '', 0, 1, valinit=0.05, valstep=0.01)
        self.slider_roll_kd = Slider(ax_roll_kd, '', 0, 2, valinit=0.5, valstep=0.05)
        
        ax_roll_apply = plt.axes([0.70, 0.40, 0.25, 0.03])
        self.btn_roll_apply = Button(ax_roll_apply, 'Aplicar PID Roll', color='lightblue')
        self.btn_roll_apply.on_clicked(self._apply_pid_roll)
        
        # PID Pitch
        self.ax_controls.text(0.5, 0.315, 'PID PITCH', 
                             ha='center', fontsize=11, fontweight='bold')
        
        ax_pitch_kp = plt.axes([0.70, 0.29, 0.25, 0.02])
        ax_pitch_ki = plt.axes([0.70, 0.24, 0.25, 0.02])
        ax_pitch_kd = plt.axes([0.70, 0.19, 0.25, 0.02])
        
        self.ax_controls.text(0.2, 0.275, 'Kp:', ha='right', fontsize=9)
        self.ax_controls.text(0.2, 0.215, 'Ki:', ha='right', fontsize=9)
        self.ax_controls.text(0.2, 0.155, 'Kd:', ha='right', fontsize=9)
        
        self.slider_pitch_kp = Slider(ax_pitch_kp, '', 0, 5, valinit=1.0, valstep=0.1)
        self.slider_pitch_ki = Slider(ax_pitch_ki, '', 0, 1, valinit=0.05, valstep=0.01)
        self.slider_pitch_kd = Slider(ax_pitch_kd, '', 0, 2, valinit=0.5, valstep=0.05)
        
        ax_pitch_apply = plt.axes([0.70, 0.15, 0.25, 0.03])
        self.btn_pitch_apply = Button(ax_pitch_apply, 'Aplicar PID Pitch', color='lightblue')
        self.btn_pitch_apply.on_clicked(self._apply_pid_pitch)
        
        # Información de telemetría
        self.ax_controls.text(0.5, 0.08, 'TELEMETRÍA', 
                             ha='center', fontsize=11, fontweight='bold')
        
        self.telemetry_text = self.ax_controls.text(0.05, 0.03, '', 
                                                    fontsize=8, verticalalignment='top',
                                                    family='monospace')
    
    def _on_throttle_change(self, val):
        """Callback del slider de throttle"""
        self.controller.set_throttle(int(val))
    
    def _on_max_throttle_change(self, val):
        """Callback del slider de max throttle"""
        self.controller.set_max_throttle(int(val))
    
    def _apply_pid_roll(self, event):
        """Aplica los valores PID de roll"""
        kp = self.slider_roll_kp.val
        ki = self.slider_roll_ki.val
        kd = self.slider_roll_kd.val
        self.controller.set_pid_roll(kp, ki, kd)
    
    def _apply_pid_pitch(self, event):
        """Aplica los valores PID de pitch"""
        kp = self.slider_pitch_kp.val
        ki = self.slider_pitch_ki.val
        kd = self.slider_pitch_kd.val
        self.controller.set_pid_pitch(kp, ki, kd)
    
    def _update_plots(self, frame):
        """Actualiza los gráficos en cada frame"""
        # Actualizar estado de conexión
        if self.controller.connected:
            status = '🟢 Conectado'
            if self.controller.armed:
                status += ' | ⚠ ARMADO'
        else:
            status = '🔴 Desconectado'
        self.status_text.set_text(status)
        
        # Si no hay datos, no actualizar gráficos
        if len(self.controller.time_data) == 0:
            return
        
        time_array = np.array(self.controller.time_data)
        
        # Actualizar Roll y Pitch
        self.line_roll.set_data(time_array, np.array(self.controller.roll_data))
        self.line_pitch.set_data(time_array, np.array(self.controller.pitch_data))
        self.ax_orientation.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar Altura
        self.line_altitude.set_data(time_array, np.array(self.controller.altitude_data))
        self.ax_altitude.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar Motores
        self.line_motor_fl.set_data(time_array, np.array(self.controller.motor_fl_data))
        self.line_motor_fr.set_data(time_array, np.array(self.controller.motor_fr_data))
        self.line_motor_bl.set_data(time_array, np.array(self.controller.motor_bl_data))
        self.line_motor_br.set_data(time_array, np.array(self.controller.motor_br_data))
        self.ax_motors.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar Throttle
        self.line_throttle.set_data(time_array, np.array(self.controller.throttle_data))
        self.ax_throttle.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar telemetría
        telemetry = f"""
Roll:     {self.controller.roll:6.2f}°
Pitch:    {self.controller.pitch:6.2f}°
Altura:   {self.controller.altitude:6.2f} m
Throttle: {self.controller.throttle:3d}

Motores:
  FL: {self.controller.motor_fl:3d}
  FR: {self.controller.motor_fr:3d}
  BL: {self.controller.motor_bl:3d}
  BR: {self.controller.motor_br:3d}
        """
        self.telemetry_text.set_text(telemetry.strip())
    
    def show(self):
        """Muestra la interfaz"""
        plt.show()

# ============================================
# FUNCIÓN PRINCIPAL
# ============================================
def main():
    """Función principal del programa"""
    print("=" * 60)
    print("  Sistema de Control de Dron ESP32")
    print("=" * 60)
    print()
    
    # Crear controlador
    controller = DroneController()
    
    # Intentar conectar
    if not controller.connect():
        print("\n⚠ No se pudo conectar. Saliendo...")
        return
    
    print("\n✓ Iniciando interfaz gráfica...")
    print("  - Cierra la ventana para salir")
    print("  - Presiona ARM para habilitar motores")
    print("  - Ajusta Throttle para controlar potencia")
    print()
    
    # Crear y mostrar GUI
    try:
        gui = DroneGUI(controller)
        gui.show()
    except KeyboardInterrupt:
        print("\n\nInterrumpido por usuario")
    finally:
        controller.disconnect()
        print("Programa terminado")

# ============================================
# PUNTO DE ENTRADA
# ============================================
if __name__ == "__main__":
    main()
