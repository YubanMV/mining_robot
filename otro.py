import cv2
import pygame
import threading
import busio
# from adafruit_pca9685 import PCA9685
# from adafruit_servokit import ServoKit
# from board import SCL, SDA

# Resto del código...

# Crear variables globales para controlar los motores
motor_lock = threading.Lock()
left_motor_speed = 0
right_motor_speed = 0

# Función para controlar los motores de forma continua en un hilo separado
def motor_control_thread():
    global left_motor_speed
    global right_motor_speed

    while True:
        # Controlar los motores con los valores actuales de velocidad
        with motor_lock:
            LeftMotorFront.duty_cycle = left_motor_speed
            RightMotorFront.duty_cycle = right_motor_speed
            LeftMotorReverse.duty_cycle = 0
            RightMotorReverse.duty_cycle = 0

        # Esperar un breve período de tiempo antes de actualizar los motores nuevamente
        time.sleep(0.1)

# Crear el hilo para controlar los motores
motor_thread = threading.Thread(target=motor_control_thread)
motor_thread.start()

# Resto del código...

# Función para leer y mover carro con joystick
def control_joystick():
    global left_motor_speed
    global right_motor_speed

    left_joystick_y = get_left_joystick_y()
    right_joystick_x = get_right_joystick_x()

    # Actualizar las velocidades de los motores según los valores del joystick
    if left_joystick_y > 0:
        # Ajustar el ancho del pulso PWM de los motores para aumentar la velocidad hacia adelante
        speed = intp(left_joystick_y)
        with motor_lock:
            left_motor_speed = speed
            right_motor_speed = speed

    elif left_joystick_y < 0:
        speed = intp(left_joystick_y)
        with motor_lock:
            left_motor_speed = 0
            right_motor_speed = 0
            LeftMotorReverse.duty_cycle = speed
            RightMotorReverse.duty_cycle = speed

    if right_joystick_x > 0:
        speed = intp(right_joystick_x)
        with motor_lock:
            left_motor_speed = speed
            right_motor_speed = 0

    elif right_joystick_x < 0:
        speed = intp(right_joystick_x)
        with motor_lock:
            left_motor_speed = 0
            right_motor_speed = speed

    # Resto del código...

# Resto del código...
