import cv2
import pygame
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
from board import SCL, SDA
import time
import threading
# Inicializar la comunicación I2C
i2c = busio.I2C(SCL, SDA)

# Inicializar el controlador PCA9685
pca = PCA9685(i2c)
pca.frequency = 50

# Inicializar el kit de servos
kit = ServoKit(channels=16)

# Asignar los canales de los motores y servos
LeftMotorFront = pca.channels[0]
LeftMotorReverse = pca.channels[1]
RightMotorFront = pca.channels[2]
RightMotorReverse = pca.channels[3]
DiggerFront = pca.channels[10]
DiggerReverse = pca.channels[11]
DrillerClock = pca.channels[12]
DrillerCounterclk = pca.channels[13]
# Inicializar Pygame
pygame.init()
pygame.display.init()
pygame.joystick.init()

# Inicializar joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()



# Variables para controlar la lectura de los valores de los joysticks
read_joystick_values = True

# Variables para el estado previo del botón 4
button_4_last_state = False


def detect_balls(frame, cascade):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    balls = cascade.detectMultiScale(gray, 1.05, 3)
    return balls

def classify_ball_color(roi):
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_red = (0, 100, 100)
    upper_red = (10, 255, 255)
    lower_green = (50, 100, 100)
    upper_green = (70, 255, 255)
    lower_blue = (110, 100, 100)
    upper_blue = (130, 255, 255)
    mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_roi, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv_roi, lower_blue, upper_blue)
    red_pixels = cv2.countNonZero(mask_red)
    green_pixels = cv2.countNonZero(mask_green)
    blue_pixels = cv2.countNonZero(mask_blue)
    if red_pixels > green_pixels and red_pixels > blue_pixels:
        return 'Red', (0, 0, 255)
    elif green_pixels > red_pixels and green_pixels > blue_pixels:
        return 'Green', (0, 255, 0)
    elif blue_pixels > red_pixels and blue_pixels > green_pixels:
        return 'Blue', (255, 0, 0)
    else:
        return None, None
    
def calculate_distance(focal_length, real_width, pixel_width):
    distance = (real_width * focal_length) / pixel_width
    return distance

def draw_ball_contour(frame, balls, color, color_bgr, center_x):
    min_distance = float('inf')
    closest_ball = None
    
    for (x, y, w, h) in balls:
        centroid_x = int(x + (w / 2))
        centroid_y = int(y + (h / 2))
        
        if x <= center_x <= x + w:
            distance = abs(centroid_x - center_x)
            if distance < min_distance:
                min_distance = distance
                closest_ball = (x, y, w, h, centroid_x, centroid_y)
    
    if closest_ball is not None:
        x, y, w, h, centroid_x, centroid_y = closest_ball
        cv2.rectangle(frame, (x, y), (x+w, y+h), color_bgr, 2)
        cv2.putText(frame, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color_bgr, 2)
        cv2.circle(frame, (centroid_x, centroid_y), 3, color_bgr, -1)
        cv2.putText(frame, f"Centroide: ({centroid_x}, {centroid_y})", (x, y+h+40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
    
    return frame

def draw_lines(frame):
    height, width = frame.shape[:2]
    center_x = int(width / 2)
    center_y = int(height / 2)
    rect_width = 100
    rect_height = 80
    x1 = center_x - int(rect_width / 2)
    y1 = center_y - int(rect_height / 2)
    x2 = center_x + int(rect_width / 2)
    y2 = center_y + int(rect_height / 2)
    cv2.line(frame, (x1, 0), (x1, center_y*2),  (233, 193, 133), 2)
    cv2.line(frame, (x2, 0), (x2, center_y*2),  (233, 193, 133), 2)
    # cv2.line(frame, (x1, y1), (x2, y1),  (233, 193, 133), 2)
    # cv2.line(frame, (x2, y2), (x1, y2),  (233, 193, 133), 2)
    
    return center_x

# Interp Function
def intp(inp):
    return abs(round(inp * 50000))

# Función para obtener los valores del joystick izquierdo (eje Y) cuando se mueve de arriba a abajo
def get_left_joystick_y():
    pygame.event.pump()
    value = -joystick.get_axis(1)  # Obtener el valor del eje Y del joystick izquierdo
    return round(value, 2)  # Redondear el valor a 2 decimales

# Función para obtener los valores del joystick derecho (eje X)
def get_right_joystick_x():
    pygame.event.pump()
    value = joystick.get_axis(2)  # Obtener el valor del eje X del joystick derecho
    return round(value, 2)  # Redondear el valor a 2 decimales

# Función para leer el estado del botón 4 del control
def get_button_4_state():
    pygame.event.pump()
    return joystick.get_button(3)  # Obtener el estado del botón 4 (índice 3) del control

# Crear variables globales para controlar los motores
motor_lock = threading.Lock()
left_motor_speed = 0
right_motor_speed = 0
left_back_motor_speed = 0
right_back_motor_speed = 0

# Función para controlar los motores de forma continua en un hilo separado
def motor_control_thread():
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    
    while True:
        # Controlar los motores con los valores actuales de velocidad
        with motor_lock:
            LeftMotorFront.duty_cycle = left_motor_speed
            RightMotorFront.duty_cycle = right_motor_speed
            LeftMotorReverse.duty_cycle = left_back_motor_speed
            RightMotorReverse.duty_cycle = right_back_motor_speed



        # Esperar un breve período de tiempo antes de actualizar los motores nuevamente
        time.sleep(0.1)

# Crear el hilo para controlar los motores
motor_thread = threading.Thread(target=motor_control_thread)
motor_thread.start()

def control_joystick():
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    
    left_joystick_y = get_left_joystick_y()
    right_joystick_x = get_right_joystick_x()

    # Actualizar las velocidades de los motores según los valores del joystick
    if left_joystick_y > 0:
        # Ajustar el ancho del pulso PWM de los motores para aumentar la velocidad hacia adelante
        speedy = intp(left_joystick_y)
        with motor_lock:
            left_motor_speed = speedy
            right_motor_speed = speedy
            left_back_motor_speed =0 
            right_back_motor_speed = 0
            
    elif left_joystick_y < 0:   
        speedy = intp(left_joystick_y)
        with motor_lock:
            left_motor_speed = 0
            right_motor_speed = 0
            left_back_motor_speed = speedy 
            right_back_motor_speed = speedy

    if right_joystick_x > 0:
        speed = intp(right_joystick_x)
        with motor_lock:
            left_motor_speed = 0
            right_motor_speed = speed
            left_back_motor_speed = speed
            right_back_motor_speed = 0

    elif right_joystick_x < 0:
        speed = intp(right_joystick_x)
        with motor_lock:
            left_motor_speed = speed
            right_motor_speed = 0
            left_back_motor_speed = 0
            right_back_motor_speed = speed
    if right_joystick_x == 0 and left_joystick_y == 0:
        with motor_lock:       
            left_motor_speed = 0
            right_motor_speed = 0
            left_back_motor_speed = 0
            right_back_motor_speed = 0 


# Función para girar a la derecha
def turn_right(frame):
    print("Girando a la derecha")
    cv2.putText(frame, "Right", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    with motor_lock:
        left_motor_speed = 0
        right_motor_speed = 65535
        left_back_motor_speed =65535
        right_back_motor_speed = 0

def turn_left(frame):
    print("Girando a la izquierda")
    cv2.putText(frame, "Left", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    with motor_lock:
        left_motor_speed = 65535
        right_motor_speed = 0
        left_back_motor_speed =0 
        right_back_motor_speed = 65535 

def forward(frame):
    print("Moviéndose hacia adelante")
    cv2.putText(frame, "Forward", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    with motor_lock:
        left_motor_speed = 65535
        right_motor_speed = 65535
        left_back_motor_speed =0 
        right_back_motor_speed = 0
def stop(frame):
    print("Detenido")
    cv2.putText(frame, "Stop", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    global left_motor_speed
    global right_motor_speed
    global left_back_motor_speed
    global right_back_motor_speed
    with motor_lock:
        left_motor_speed = 0
        right_motor_speed = 0
        left_back_motor_speed =0 
        right_back_motor_speed = 0

# Función para mostrar los valores de los joysticks, el estado del botón 4 y la transmisión de la cámara en la terminal
support = 1
def show_joystick_values_and_camera():
    cascade = cv2.CascadeClassifier('cascade.xml')
    # Inicializar cámara
    capture = cv2.VideoCapture(0)
    global read_joystick_values
    global button_4_last_state
    global support
    focal_length = 405  # Focal length in pixels
    real_width = 4  # Real width of the balls in cm 
    no_ball_count = 0  # Contador para llevar el registro de cuántos fotogramas no se detecta ninguna pelota

    print("Llego aki")
    while True:
        # Obtener el fotograma de la cámara
        ret, frame = capture.read()

        center_x = draw_lines(frame)
        balls = detect_balls(frame, cascade)
        if len(balls) == 0 and support == 0:
            no_ball_count += 1
            if no_ball_count > 10:  # Si no se detecta ninguna pelota durante más de 10 fotogramas
                turn_left(frame)
        else:
            no_ball_count = 0  # Reiniciar el contador si se detecta alguna pelota

        for (x, y, w, h) in balls:
            if support == 1:
                if x <= center_x <= x + w:
                    roi = frame[y:y+h, x:x+w]
                    color, color_bgr = classify_ball_color(roi)

                    if color is not None:
                        frame = draw_ball_contour(frame, [(x, y, w, h)], color, color_bgr, center_x)
                        ball_color = color
                        pixel_width = w  # Ancho de la pelota en píxeles
                        distance = calculate_distance(focal_length, real_width, pixel_width)
                        distance_text = 'Distancia: {:.2f} cm'.format(distance)
                        cv2.putText(frame, distance_text, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)

            if support == 0:
                colors_bgr = {
                    'Red': (0, 0, 255),
                    'Green': (0, 255, 0),
                    'Blue': (255, 0, 0)
                }
                color, color_bgr = classify_ball_color(frame[y:y+h, x:x+w])
                if color == ball_color:
                    frame = draw_ball_contour(frame, [(x, y, w, h)], color, colors_bgr[color], center_x)                                           
                    pixel_width = w  # Ancho de la pelota en píxeles
                    distance = calculate_distance(focal_length, real_width, pixel_width)
                    distance_text = 'Distancia: {:.2f} cm'.format(distance)
                    cv2.putText(frame, distance_text, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
                    if x + w < center_x:
                        turn_left(frame)
                    elif x > center_x:
                        turn_right(frame)
                    else:
                        if distance >= 18:
                            forward(frame)
                        else:
                            stop(frame)

        cv2.imshow('Ball Detection, Classification, and Distance', frame)
      
      
        # Leer los valores de los joysticks solo si read_joystick_values es True
        if read_joystick_values:
            # Obtener los valores del joystick izquierdo (eje Y) y derecho (eje X)
            control_joystick()
            support = 1

            # for (x, y, w, h) in balls:
                    
            #     frame = draw_ball_contour(frame, [(x, y, w, h)], ball_color, colors_bgr[ball_color], center_x)
            #     pixel_width = w  # Width of the ball in pixels
            #     distance = calculate_distance(focal_length, real_width, pixel_width)
            #     distance_text = 'Distance: {:.2f} cm'.format(distance)
            #     cv2.putText(frame, distance_text, (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)

            

        # Obtener el estado del botón 4
        button_4_state = get_button_4_state()

        # Mostrar el estado del botón 4 en la terminal
        # print(f'Button 4 State: {button_4_state}')

        # Mostrar la transmisión de la cámara
        # Cambiar el estado de read_joystick_values si se presiona o se suelta el botón 4
        if button_4_state != button_4_last_state:
            if button_4_state:  # Botón 4 se ha presionado
                support = 0
                read_joystick_values = not read_joystick_values
            button_4_last_state = button_4_state

        # Salir si se presiona la tecla 'Esc'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # Liberar recursos
    capture.release()
    cv2.destroyAllWindows()

# Ejecutar el programa
if __name__ == '__main__':
    show_joystick_values_and_camera()
