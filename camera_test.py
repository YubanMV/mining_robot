import cv2
import numpy as np

# Parámetros de la cámara y el objeto
distancia_focal = 445# Inserta la distancia focal de tu cámara en píxeles
ancho_real_objeto = 0.04# Inserta el ancho real del objeto en unidades de medida (por ejemplo, metros)

# Cargar el modelo XML
cascade = cv2.CascadeClassifier('cascade_real.xml')

# Inicializar la cámara
cap = cv2.VideoCapture(0)  # Puedes cambiar el número para seleccionar una cámara diferente

while True:
    # Leer el cuadro actual de la cámara
    ret, frame = cap.read()
    
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detectar objetos en el cuadro
    objects = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    
    for (x, y, w, h) in objects:
        # Calcular la distancia del objeto a la cámara
        distancia = (ancho_real_objeto * distancia_focal) / w
        
        # Dibujar un rectángulo alrededor del objeto
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Mostrar la distancia sobre el rectángulo
        cv2.putText(frame, f"Distancia: {distancia:.2f} unidades", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    
    # Mostrar el cuadro procesado
    cv2.imshow('Distance Measurement', frame)
    
    # Detener el bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
