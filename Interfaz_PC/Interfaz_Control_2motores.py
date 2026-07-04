"""
Sistema de Control y Monitoreo de Dron - Python (2 MOTORES)
===========================================================

Interfaz gráfica para controlar el dron ESP32 vía WiFi.
Versión adaptada para 2 motores (Izquierdo y Derecho).

- Conexión WiFi al dron
- Envío de comandos (ARM, THROTTLE, PID)
- Visualización en tiempo real de telemetría
- Gráficos de roll y potencia de 2 motores
- Ajuste de parámetros PID para roll

"""

import socket
import threading
import time
import sys
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button, Slider
import numpy as np

# ============================================
# CONFIGURACIÓN DE CONEXIÓN
# ============================================
DRONE_IP = "192.168.4.1"  # IP del Access Point de la ESP32
DRONE_PORT = 8888
TIMEOUT = 5.0

# ============================================
# CLASE PRINCIPAL DEL CONTROLADOR (2 MOTORES)
# ============================================
class DroneController:
    def __init__(self):
        """Inicializa el controlador del dron"""
        self.socket = None
        self.connected = False
        self.running = False
        
        # Variables de telemetría (2 motores)
        self.roll = 0.0
        self.throttle = 0
        self.motor_left = 0
        self.motor_right = 0
        
        # Historial para gráficos (últimos 200 puntos)
        self.max_points = 200
        self.time_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.throttle_data = deque(maxlen=self.max_points)
        self.motor_left_data = deque(maxlen=self.max_points)
        self.motor_right_data = deque(maxlen=self.max_points)
        
        self.start_time = time.time()
        
        # Estado del sistema
        self.armed = False
        self.max_throttle = 100
        
        # PID actuales
        self.pid_roll = {"kp": 1.5, "ki": 0.08, "kd": 0.6}
        
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
            
            # Reiniciar tiempo de referencia para gráficas
            self.start_time = time.time()
            
            # Limpiar historial de datos para evitar conflictos de timestamp
            self.time_data.clear()
            self.roll_data.clear()
            self.throttle_data.clear()
            self.motor_left_data.clear()
            self.motor_right_data.clear()
            
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
            # Formato para 2 motores: TELEMETRY:roll,throttle,motor_left,motor_right
            try:
                parts = message.split(':')[1].split(',')
                
                if len(parts) == 4:
                    self.roll = float(parts[0])
                    self.throttle = int(parts[1])
                    self.motor_left = int(parts[2])
                    self.motor_right = int(parts[3])
                else:
                    print(f"Formato de telemetría desconocido: {len(parts)} campos")
                    return

                # Agregar a historial
                current_time = time.time() - self.start_time
                self.time_data.append(current_time)
                self.roll_data.append(self.roll)
                self.throttle_data.append(self.throttle)
                self.motor_left_data.append(self.motor_left)
                self.motor_right_data.append(self.motor_right)
                
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
    
    def request_status(self):
        """Solicitar estado actual del dron"""
        self.send_command("STATUS")
    
    def calibrate(self):
        """Calibrar el MPU6050 (mantener el dron quieto)"""
        if self.send_command("CALIBRATE"):
            print("⚙ Calibrando MPU6050... mantén el dron quieto")


# ============================================
# INTERFAZ GRÁFICA CON MATPLOTLIB (2 MOTORES)
# ============================================
class DroneGUI:
    def __init__(self, controller):
        """Inicializa la interfaz gráfica"""
        self.controller = controller
        
        # Crear figura con múltiples subplots
        self.fig = plt.figure(figsize=(14, 9))
        self.fig.canvas.manager.set_window_title('Control de Dron ESP32 (2 Motores)')
        
        try:
            manager = plt.get_current_fig_manager()
            manager.window.state('zoomed')
        except:
            pass  # Si falla, continuar con el tamaño normal
        
        # Configurar grid (2x2 para gráficos + columna de controles)
        gs = self.fig.add_gridspec(2, 3, hspace=0.4, wspace=0.3)

        # UBICA LOS 3 GRÁFICOS
        self.ax_motors = self.fig.add_subplot(gs[0, 0])
        self.ax_throttle = self.fig.add_subplot(gs[1, 0])
        self.ax_orientation = self.fig.add_subplot(gs[:, 1])  # Span both rows
        
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
        # Roll
        self.ax_orientation.set_title('Orientación (Roll)', fontsize=12, fontweight='bold')
        self.ax_orientation.set_ylabel('Ángulo (°)')
        self.ax_orientation.set_xlabel('Tiempo (s)')
        self.ax_orientation.grid(True, alpha=0.3)
        self.ax_orientation.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.line_roll, = self.ax_orientation.plot([], [], 'b-', label='Roll', linewidth=2)
        self.ax_orientation.legend(loc='upper right')
        self.ax_orientation.set_ylim(-45, 45)
        
        # Motores (2 motores)
        self.ax_motors.set_title('Potencia de Motores (2 Motores)', fontsize=12, fontweight='bold')
        self.ax_motors.set_ylabel('PWM (0-255)')
        self.ax_motors.set_xlabel('Tiempo (s)')
        self.ax_motors.grid(True, alpha=0.3)
        self.line_motor_left, = self.ax_motors.plot([], [], 'g-', label='Izquierdo', linewidth=2)
        self.line_motor_right, = self.ax_motors.plot([], [], 'r-', label='Derecho', linewidth=2)
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
        """Panel derecho de controles"""
        
        # Título
        self.ax_controls.text(.75, 1.1, 'PANEL DE CONTROL', 
                             ha='center', va='top', fontsize=14, fontweight='bold',
                             transform=self.ax_controls.transAxes)
        
        # Estado de conexión
        self.status_text = self.ax_controls.text(.75, 1.05, '🔴 Desconectado', 
                                                ha='center', va='top', fontsize=10,
                                                transform=self.ax_controls.transAxes,
                                                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
        
        # Botones Conectar / Desconectar
        ax_conn = self.fig.add_axes([0.73, 0.84, 0.10, 0.035])
        ax_disc = self.fig.add_axes([0.85, 0.84, 0.10, 0.035])
        self.btn_connect = Button(ax_conn, "Conectar", color="lightblue")
        self.btn_disconnect = Button(ax_disc, "Desconectar", color="lightcoral")
        self.btn_connect.on_clicked(self._on_connect)
        self.btn_disconnect.on_clicked(self._on_disconnect)

        # Botones ARM / DISARM
        ax_arm = self.fig.add_axes([0.73, 0.78, 0.10, 0.035])
        ax_disarm = self.fig.add_axes([0.85, 0.78, 0.10, 0.035])
        self.btn_arm = Button(ax_arm, "ARM", color="lightgreen")
        self.btn_disarm = Button(ax_disarm, "DISARM", color="lightcoral")
        self.btn_arm.on_clicked(lambda e: self.controller.arm())
        self.btn_disarm.on_clicked(lambda e: self.controller.disarm())

        # Botón Calibrar
        ax_calibrate = self.fig.add_axes([0.73, 0.72, 0.22, 0.035])
        self.btn_calibrate = Button(ax_calibrate, "Calibrar MPU6050", color="lightyellow")
        self.btn_calibrate.on_clicked(lambda e: self.controller.calibrate())

        # Slider Throttle
        ax_throttle = self.fig.add_axes([0.73, 0.64, 0.22, 0.025])
        self.slider_throttle = Slider(ax_throttle, "Throttle", 0, 255, valinit=0, color="purple")
        self.slider_throttle.on_changed(self._on_throttle_change)

        # Slider Max Throttle
        ax_max_throttle = self.fig.add_axes([0.73, 0.58, 0.22, 0.025])
        self.slider_max_throttle = Slider(ax_max_throttle, "Max Throttle", 0, 255, valinit=100, color="orange")
        self.slider_max_throttle.on_changed(self._on_max_throttle_change)

        # Sección PID ROLL
        self.ax_controls.text(0.75, 0.50, 'PID ROLL', 
                             ha='center', fontsize=11, color='blue', fontweight='bold',
                             transform=self.ax_controls.transAxes)

        ax_roll_kp = self.fig.add_axes([0.73, 0.44, 0.22, 0.025])
        self.slider_roll_kp = Slider(ax_roll_kp, "Kp", 0, 5, valinit=1.5, color="cyan")

        ax_roll_ki = self.fig.add_axes([0.73, 0.39, 0.22, 0.025])
        self.slider_roll_ki = Slider(ax_roll_ki, "Ki", 0, 1, valinit=0.08, color="cyan")

        ax_roll_kd = self.fig.add_axes([0.73, 0.34, 0.22, 0.025])
        self.slider_roll_kd = Slider(ax_roll_kd, "Kd", 0, 2, valinit=0.6, color="cyan")

        ax_apply_roll = self.fig.add_axes([0.73, 0.28, 0.22, 0.035])
        self.btn_roll_apply = Button(ax_apply_roll, "Aplicar PID Roll")
        self.btn_roll_apply.on_clicked(self._apply_pid_roll)

        # Telemetría
        self.ax_controls.text(0.75, 0.23, 'TELEMETRÍA', 
                             ha='center', fontsize=11, color='darkgreen', fontweight='bold',
                             transform=self.ax_controls.transAxes)
        
        self.telemetry_text = self.ax_controls.text(
            0.75, 0.12, '', fontsize=9, family='monospace',
            ha='center', va='top', transform=self.ax_controls.transAxes,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.9))

    def _on_connect(self, event):
        """Conecta al dron"""
        if not self.controller.connected:
            print("\n[+] Intentando conectar...")
            self.controller.connect()
    
    def _on_disconnect(self, event):
        """Desconecta del dron"""
        if self.controller.connected:
            print("\n[-] Desconectando...")
            self.controller.disconnect()
    
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
        
        # Actualizar Roll
        self.line_roll.set_data(time_array, np.array(self.controller.roll_data))
        self.ax_orientation.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar Motores (2 motores)
        self.line_motor_left.set_data(time_array, np.array(self.controller.motor_left_data))
        self.line_motor_right.set_data(time_array, np.array(self.controller.motor_right_data))
        self.ax_motors.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar Throttle
        self.line_throttle.set_data(time_array, np.array(self.controller.throttle_data))
        self.ax_throttle.set_xlim(max(0, time_array[-1] - 20), time_array[-1] + 1)
        
        # Actualizar telemetría
        telemetry = f"""
Roll:     {self.controller.roll:6.2f}°
Throttle: {self.controller.throttle:3d}

Motores:
  Izquierdo:  {self.controller.motor_left:3d}
  Derecho:    {self.controller.motor_right:3d}
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
    print("  Sistema de Control de Dron ESP32 (2 MOTORES)")
    print("=" * 60)
    print()
    
    # Crear controlador (sin conectar aún)
    controller = DroneController()
    
    print("[*] Iniciando interfaz gráfica...")
    print("  - Usa el botón CONECTAR para conectarte al dron")
    print("  - Cierra la ventana para salir")
    print()
    
    # Crear y mostrar GUI
    try:
        gui = DroneGUI(controller)
        gui.show()
    except KeyboardInterrupt:
        print("\n\nInterrumpido por usuario")
    finally:
        if controller.connected:
            controller.disconnect()
        print("Programa terminado")

if __name__ == "__main__":
    main()
