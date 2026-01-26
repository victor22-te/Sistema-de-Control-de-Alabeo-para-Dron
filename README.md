# Proyecto Dron - Control Clásico

Este proyecto contiene el código para el control de un dron utilizando ESP32 y sensores MPU6050.

## Archivos Principales

### Interfaz_Dron/
- **esp_2motores.py**: Script para el ESP32 que controla 2 motores del dron
- **Interfaz_Control_2motores.py**: Interfaz de control para monitorear y controlar el dron con 2 motores

## Requisitos

- Python 3.x
- ESP32
- Sensor MPU6050
- Bibliotecas Python necesarias (ver requirements.txt si está disponible)

## Uso

1. Cargar el código `esp_2motores.py` en el ESP32
2. Ejecutar `Interfaz_Control_2motores.py` en la computadora para controlar el dron

## Estructura del Proyecto

```
Proyecto Dron/
├── Interfaz_Dron/          # Código principal de control
├── lecturaMPU/             # Scripts para lectura del sensor MPU6050
├── Simple PID/             # Implementación de control PID
├── Codigos1.0/             # Versiones anteriores del código
└── MPU6050 python/         # Utilidades para MPU6050
```

## Notas

Este proyecto es parte del curso de Control Clásico.
