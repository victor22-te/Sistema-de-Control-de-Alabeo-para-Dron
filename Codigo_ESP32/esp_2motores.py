"""
Sistema de Control de Vuelo para Dron - ESP32 MicroPython (2 MOTORES)
=====================================================================
Funcionalidades:
- Lectura y filtrado de datos del MPU6050
- Control PID para roll (estabilización izquierda-derecha)
- Comunicación WiFi para control remoto
- Debug por Serial (roll, potencias PWM de 2 motores)
"""

from machine import Pin, PWM, I2C
import network
import socket
import time
import math
import _thread
import sys

# ============================================
# CONFIGURACIÓN DE PINES (2 MOTORES)
# ============================================
MOTOR_LEFT_PIN = 25   # Motor Izquierdo
MOTOR_RIGHT_PIN = 26  # Motor Derecho

# Pines I2C para MPU6050
SDA_PIN = 21
SCL_PIN = 22

# ============================================
# CONFIGURACIÓN WIFI
# ============================================
WIFI_SSID = "DroneControl"
WIFI_PASSWORD = "drone1234"
SERVER_PORT = 8888

# ============================================
# CONFIGURACIÓN MPU6050
# ============================================
MPU6050_ADDR = 0x68

# Registros del MPU6050
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
CONFIG = 0x1A

# ============================================
# CLASE MPU6050
# ============================================
class MPU6050:
    def __init__(self, i2c, addr=MPU6050_ADDR):
        self.i2c = i2c
        self.addr = addr
        
        # Offsets de calibración
        self.accel_offset = [0, 0, 0]
        self.gyro_offset = [0, 0, 0]
        
        # Despertar el sensor
        self.i2c.writeto_mem(self.addr, PWR_MGMT_1, bytes([0]))
        time.sleep_ms(100)
        
        # Configurar rango acelerómetro (±8g)
        self.i2c.writeto_mem(self.addr, ACCEL_CONFIG, bytes([0x10]))
        
        # Configurar rango giroscopio (±500°/s)
        self.i2c.writeto_mem(self.addr, GYRO_CONFIG, bytes([0x08]))
        
        # Configurar filtro pasa-bajos (42Hz)
        self.i2c.writeto_mem(self.addr, CONFIG, bytes([0x03]))
        
        print("[MPU6050] Inicializado")
    
    def read_raw_data(self):
        """Lee los datos crudos del sensor"""
        data = self.i2c.readfrom_mem(self.addr, ACCEL_XOUT_H, 14)
        
        # Convertir bytes a valores de 16 bits con signo
        accel_x = self._bytes_to_int(data[0], data[1])
        accel_y = self._bytes_to_int(data[2], data[3])
        accel_z = self._bytes_to_int(data[4], data[5])
        gyro_x = self._bytes_to_int(data[8], data[9])
        gyro_y = self._bytes_to_int(data[10], data[11])
        gyro_z = self._bytes_to_int(data[12], data[13])
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    
    def _bytes_to_int(self, high, low):
        """Convierte dos bytes a entero de 16 bits con signo"""
        value = (high << 8) | low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    
    def calibrate(self, samples=2000):
        """Calibra el sensor (mantener quieto)"""
        print("[MPU6050] Calibrando... mantén el dron quieto")
        
        sum_accel = [0, 0, 0]
        sum_gyro = [0, 0, 0]
        
        for i in range(samples):
            ax, ay, az, gx, gy, gz = self.read_raw_data()
            sum_accel[0] += ax
            sum_accel[1] += ay
            sum_accel[2] += az
            sum_gyro[0] += gx
            sum_gyro[1] += gy
            sum_gyro[2] += gz
            
            if i % 500 == 0:
                print(f"  Progreso: {i}/{samples}")
            
            time.sleep_ms(2)
        
        # Calcular offsets
        self.accel_offset[0] = sum_accel[0] / samples
        self.accel_offset[1] = sum_accel[1] / samples
        self.accel_offset[2] = (sum_accel[2] / samples) - 4096  # Restar 1g
        
        self.gyro_offset[0] = sum_gyro[0] / samples
        self.gyro_offset[1] = sum_gyro[1] / samples
        self.gyro_offset[2] = sum_gyro[2] / samples
        
        print("[MPU6050] Calibración completa")
    
    def get_calibrated_data(self):
        """Lee datos calibrados en unidades físicas"""
        ax, ay, az, gx, gy, gz = self.read_raw_data()
        
        # Aplicar offsets y convertir a unidades físicas
        # Acelerómetro: ±8g → 4096 LSB/g
        accel_x = (ax - self.accel_offset[0]) / 4096.0
        accel_y = (ay - self.accel_offset[1]) / 4096.0
        accel_z = (az - self.accel_offset[2]) / 4096.0
        
        # Giroscopio: ±500°/s → 65.5 LSB/(°/s)
        gyro_x = (gx - self.gyro_offset[0]) / 65.5
        gyro_y = (gy - self.gyro_offset[1]) / 65.5
        gyro_z = (gz - self.gyro_offset[2]) / 65.5
        
        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# ============================================
# CLASE CONTROLADOR PID
# ============================================
class PIDController:
    def __init__(self, kp, ki, kd, max_integral=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        
        self.last_error = 0
        self.integral = 0
    
    def compute(self, error, dt):
        """Calcula la salida del PID"""
        # Proporcional
        p_term = self.kp * error
        
        # Integral (con anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        i_term = self.ki * self.integral
        
        # Derivativo
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0
        
        self.last_error = error
        
        return p_term + i_term + d_term
    
    def reset(self):
        """Reinicia el PID"""
        self.last_error = 0
        self.integral = 0
    
    def set_gains(self, kp, ki, kd):
        """Actualiza las ganancias"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

# ============================================
# CLASE PRINCIPAL DEL DRON (2 MOTORES)
# ============================================
class DroneController:
    def __init__(self):
        print("\n" + "="*50)
        print("  Sistema de Control de Dron - ESP32 (2 MOTORES)")
        print("="*50 + "\n")
        
        # Inicializar I2C y MPU6050
        self.i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
        self.mpu = MPU6050(self.i2c)
        self.mpu.calibrate()
        
        # Inicializar motores (PWM) - Solo 2 motores
        # ESCs requieren 50Hz (período de 20ms) con pulsos de 1000-2000μs
        self.motor_left = PWM(Pin(MOTOR_LEFT_PIN), freq=50, duty=0)
        self.motor_right = PWM(Pin(MOTOR_RIGHT_PIN), freq=50, duty=0)
        print("[Motores] Inicializados (2 motores: Izquierdo y Derecho) - 50Hz para ESCs")
        
        # Controlador PID (solo roll para 2 motores)
        self.pid_roll = PIDController(1.5, 0.08, 0.6)
        
        # Variables de orientación
        self.roll = 0.0  # Solo roll para 2 motores
        self.alpha = 0.96  # Filtro complementario
        
        # Variables de control
        self.armed = False
        self.throttle = 0
        self.max_throttle = 100
        self.idle_throttle = 170  # Potencia inicial al armar (0-255)
        
        # Valores PWM de motores (0-1023 en MicroPython)
        self.motor_left_pwm = 0
        self.motor_right_pwm = 0
        
        # Timing
        self.last_time = time.ticks_ms()
        
        # WiFi y socket
        self.client = None
        self.setup_wifi()
        
        # Flag para debug
        self.debug_enabled = True
        self.last_debug_time = time.ticks_ms()
        
        # Buffer para comandos
        self.buffer = ""
        
    def setup_wifi(self):
        """Configura WiFi como Access Point"""
        self.ap = network.WLAN(network.AP_IF)
        self.ap.active(True)
        self.ap.config(essid=WIFI_SSID, password=WIFI_PASSWORD)
        
        while not self.ap.active():
            time.sleep_ms(100)
        
        print(f"[WiFi] Access Point: {WIFI_SSID}")
        print(f"[WiFi] IP: {self.ap.ifconfig()[0]}")
        
        # Crear socket servidor
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('', SERVER_PORT))
        self.server_socket.listen(1)
        self.server_socket.setblocking(False)
        
        print(f"[WiFi] Servidor en puerto {SERVER_PORT}")
        print("\n[Sistema] Listo - Esperando conexión...\n")
    
    def calculate_orientation(self, ax, ay, az, gx, gy, gz, dt):
        """Calcula roll usando filtro complementario (solo roll para 2 motores)"""
        # Ángulo del acelerómetro
        roll_accel = math.atan2(ay, az) * 180 / math.pi
        
        # Integrar giroscopio
        roll_gyro = self.roll + gx * dt
        
        # Filtro complementario
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_accel
    
    def update_motors(self):
        """Actualiza las señales PWM de los motores (2 motores)"""
        if self.armed:
            # Calcular corrección PID solo para roll
            dt = 0.01  # Aproximado, se calcula en el loop
            roll_correction = self.pid_roll.compute(-self.roll, dt)
            
            # Mezcla de motores (configuración izquierda-derecha)
            # Motor izquierdo: disminuye cuando se inclina a la izquierda (roll negativo)
            # Motor derecho: disminuye cuando se inclina a la derecha (roll positivo)
            self.motor_left_pwm = self.throttle - roll_correction
            self.motor_right_pwm = self.throttle + roll_correction
            
            # Limitar valores
            self.motor_left_pwm = max(0, min(self.max_throttle, int(self.motor_left_pwm)))
            self.motor_right_pwm = max(0, min(self.max_throttle, int(self.motor_right_pwm)))
        else:
            self.motor_left_pwm = 0
            self.motor_right_pwm = 0
            self.pid_roll.reset()
        
        # Escribir PWM para ESCs
        # ESC range: 1000μs-2000μs en período de 20ms (50Hz)
        # 1000μs = 5% de 20000μs → duty ~51 (de 1023)
        # 2000μs = 10% de 20000μs → duty ~102 (de 1023)
        min_duty = 51   # 1000μs (ESC armado, sin potencia)
        max_duty = 102  # 2000μs (ESC a máxima potencia)
        
        if self.armed and self.motor_left_pwm > 0:
            duty_left = int(min_duty + (self.motor_left_pwm / 255.0) * (max_duty - min_duty))
        else:
            duty_left = 0 if not self.armed else min_duty
        
        if self.armed and self.motor_right_pwm > 0:
            duty_right = int(min_duty + (self.motor_right_pwm / 255.0) * (max_duty - min_duty))
        else:
            duty_right = 0 if not self.armed else min_duty
        
        self.motor_left.duty(duty_left)
        self.motor_right.duty(duty_right)
    
    def print_debug(self):
        """Imprime información de debug por Serial (2 motores)"""
        if not self.debug_enabled:
            return
        
        # Imprimir cada 200ms
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_debug_time) < 200:
            return
        
        self.last_debug_time = current_time
        
        # Crear línea de debug
        status = "ARMADO" if self.armed else "DESARMADO"
        
        print(f"[{status}] Roll: {self.roll:6.2f}° | "
              f"Throttle: {self.throttle:3d} | "
              f"PWM[LEFT:{self.motor_left_pwm:3d} RIGHT:{self.motor_right_pwm:3d}]")
    
    def handle_client(self):
        """Maneja conexión de cliente WiFi"""
        # Aceptar nueva conexión si no hay cliente
        if self.client is None:
            try:
                self.client, addr = self.server_socket.accept()
                self.client.setblocking(False)
                print(f"[WiFi] Cliente conectado: {addr}")
                self.buffer = ""  # Limpiar buffer al conectar
            except OSError:
                return
        
        # Leer comandos
        try:
            data = self.client.recv(1024)
            if data:
                self.buffer += data.decode('utf-8')
                
                # Procesar líneas completas
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.process_command(line)
                        
        except OSError:
            pass  # No hay datos disponibles
        except Exception as e:
            print(f"[Error] Cliente: {e}")
            self.client.close()
            self.client = None
    
    def process_command(self, cmd):
        """Procesa un comando recibido"""
        try:
            if cmd == "ARM":
                self.armed = True
                # Establecer throttle inicial al armar
                if self.throttle == 0:
                    self.throttle = self.idle_throttle
                print(f"[CMD] Sistema ARMADO - Throttle inicial: {self.throttle}")
                self.send_response("OK:ARMED")
                
            elif cmd == "DISARM":
                self.armed = False
                self.throttle = 0
                print("[CMD] Sistema DESARMADO")
                self.send_response("OK:DISARMED")
                
            elif cmd.startswith("THROTTLE:"):
                value = int(cmd.split(':')[1])
                self.throttle = max(0, min(self.max_throttle, value))
                print(f"[CMD] Throttle = {self.throttle}")
                self.send_response(f"OK:THROTTLE={self.throttle}")
                
            elif cmd.startswith("MAXTHROTTLE:"):
                value = int(cmd.split(':')[1])
                self.max_throttle = max(0, min(255, value))
                print(f"[CMD] Max Throttle = {self.max_throttle}")
                self.send_response(f"OK:MAXTHROTTLE={self.max_throttle}")
                
            elif cmd.startswith("PID_ROLL:"):
                parts = cmd.split(':')[1].split(',')
                kp, ki, kd = float(parts[0]), float(parts[1]), float(parts[2])
                self.pid_roll.set_gains(kp, ki, kd)
                print(f"[CMD] PID Roll: Kp={kp} Ki={ki} Kd={kd}")
                self.send_response("OK:PID_ROLL")
                
            elif cmd == "CALIBRATE":
                print("[CMD] Iniciando calibración...")
                self.send_response("OK:CALIBRATING")
                # Desarmar durante calibración por seguridad
                was_armed = self.armed
                self.armed = False
                self.throttle = 0
                self.update_motors()
                # Ejecutar calibración
                self.mpu.calibrate()
                print("[CMD] Calibración completa")
                self.send_response("OK:CALIBRATION_COMPLETE")
                # Restaurar estado armado si estaba armado
                if was_armed:
                    self.armed = True
                
            elif cmd == "STATUS":
                self.send_telemetry()
                
            else:
                self.send_response("ERROR:UNKNOWN_COMMAND")
                
        except Exception as e:
            print(f"[Error] Procesando comando '{cmd}': {e}")
            self.send_response(f"ERROR:{e}")
    
    def send_response(self, msg):
        """Envía un mensaje al cliente"""
        if self.client:
            try:
                self.client.send((msg + "\n").encode())
            except:
                pass
    
    def send_telemetry(self):
        """Envía telemetría al cliente (2 motores)"""
        if self.client:
            try:
                # Formato: TELEMETRY:roll,throttle,motor_left,motor_right
                data = f"TELEMETRY:{self.roll:.2f}," \
                       f"{self.throttle}," \
                       f"{self.motor_left_pwm},{self.motor_right_pwm}\n"
                self.client.send(data.encode())
            except:
                pass
    
    def run(self):
        """Loop principal del sistema"""
        print("[Sistema] Iniciando loop principal...\n")
        
        telemetry_counter = 0
        
        while True:
            try:
                # Calcular delta time
                current_time = time.ticks_ms()
                dt = time.ticks_diff(current_time, self.last_time) / 1000.0
                self.last_time = current_time
                
                # Leer MPU6050
                ax, ay, az, gx, gy, gz = self.mpu.get_calibrated_data()
                
                # Calcular orientación
                self.calculate_orientation(ax, ay, az, gx, gy, gz, dt)
                
                # Actualizar motores
                self.update_motors()
                
                # Imprimir debug
                self.print_debug()
                
                # Manejar cliente WiFi
                self.handle_client()
                
                # Enviar telemetría cada 50ms (20Hz)
                telemetry_counter += 1
                if telemetry_counter >= 5:  # Asumiendo ~10ms por loop
                    self.send_telemetry()
                    telemetry_counter = 0
                
                # Pequeña pausa
                time.sleep_ms(10)
                
            except KeyboardInterrupt:
                print("\n[Sistema] Detenido por usuario")
                self.armed = False
                self.throttle = 0
                self.update_motors()
                break
            except Exception as e:
                print(f"[Error] Loop principal: {e}")
                sys.print_exception(e)

# ============================================
# PUNTO DE ENTRADA
# ============================================
def main():
    drone = DroneController()
    drone.run()

# Ejecutar si es el archivo principal
if __name__ == "__main__":
    main()
