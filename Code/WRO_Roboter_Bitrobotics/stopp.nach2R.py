
import cv2
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO  # GPIO-Steuerung für den Roboter
from time import sleep

# Kamera initialisieren
camera = PiCamera()
camera.resolution = (250, 140)  # Auflösung setzen
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

    # Zähle die Anzahl der "weißen" Pixel, die Linien darstellen
    line_pixels = cv2.countNonZero(edges)

    return line_pixels

# Hauptlogik für Steuerung
line_counter = 0  # Zähler für erkannte Linien

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array  # Das aktuelle Bild aus dem Buffer holen

    # Linienerkennung durchführen
    line_pixels = line_detection(image)

    # Wenn genügend Linienpixel erkannt werden, erhöhe den Zähler
    if line_pixels > 500:  # Schwellenwert, um eine Linie zu erkennen
        line_counter += 1
        print(f"Linie erkannt! Gesamtzahl: {line_counter}")

    # Wenn der Roboter 24 Linien erkannt hat, stoppe ihn
    if line_counter >= 24:
        print("24 Linien erkannt. Roboter stoppt.")
        stop()  
        break  

    # Wenn weniger als 24 Linien erkannt werden, fährt der Roboter weiter
    move_forward()

    # Buffer löschen
    rawCapture.truncate(0)

    # Wenn die Taste 'q' gedrückt wird, das Programm beenden
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# GPIO sauber beenden
GPIO.cleanup()
cv2.destroyAllWindows()
