
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO  # GPIO-Steuerung f�r den Roboter
from time import sleep

# Kamera initialisieren
camera = PiCamera()
camera.resolution = (250, 140)  # Aufl�sung setzen
camera.framerate = 32           # Framerate setzen
rawCapture = PiRGBArray(camera, size=(250, 140))  # Bildpuffer

# Ein bisschen Wartezeit, damit die Kamera sich anpassen kann
time.sleep(2)

# GPIO initialisieren
GPIO.setmode(GPIO.BOARD)



# Linienerkennung
def line_detection(frame):
    # Bild in Graustufen umwandeln
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Bild schwellenwerten, um Linien zu erkennen
    _, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

    # Finde die Kanten im Bild 
    edges = cv2.Canny(threshold, 50, 150)

    # Z�hle die Anzahl der "wei�en" Pixel, die Linien darstellen
    line_pixels = cv2.countNonZero(edges)

    return line_pixels

# Hauptlogik f�r Steuerung
line_counter = 0  # Z�hler f�r erkannte Linien

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array  # Das aktuelle Bild aus dem Buffer holen

    # Linienerkennung durchf�hren
    line_pixels = line_detection(image)

    # Wenn gen�gend Linienpixel erkannt werden, erh�he den Z�hler
    if line_pixels > 500:  # Schwellenwert, um eine Linie zu erkennen
        line_counter += 1
        print(f"Linie erkannt! Gesamtzahl: {line_counter}")

    # Wenn der Roboter 24 Linien erkannt hat, stoppe ihn
    if line_counter >= 24:
        print("24 Linien erkannt. Roboter stoppt.")
        stop()  
        break  

    # Wenn weniger als 24 Linien erkannt werden, f�hrt der Roboter weiter
    move_forward()

    # Buffer l�schen
    rawCapture.truncate(0)

    # Wenn die Taste 'q' gedr�ckt wird, das Programm beenden
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# GPIO sauber beenden
GPIO.cleanup()
cv2.destroyAllWindows()
