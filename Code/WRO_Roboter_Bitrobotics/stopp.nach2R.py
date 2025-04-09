
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

# Definiere Motoren/Pins (dies ist ein Beispiel, die Pins müssen je nach deinem Roboter angepasst werden)
LEFT_MOTOR_FORWARD = 18
LEFT_MOTOR_BACKWARD = 22
RIGHT_MOTOR_FORWARD = 23
RIGHT_MOTOR_BACKWARD = 24

# Motoren als Ausgang festlegen
GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)

# Funktionen zur Steuerung des Roboters
def move_forward():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def move_backward():
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)

def turn_left():
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def turn_right():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)

def stop():
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

# Linienerkennung
def line_detection(frame):
    # Bild in Graustufen umwandeln
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Bild schwellenwerten, um Linien zu erkennen
    _, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)

    # Finde die Kanten im Bild (optional: Kanten können auch helfen, Linien zu erkennen)
    edges = cv2.Canny(threshold, 50, 150)

    # Zähle die Anzahl der "weißen" Pixel, die Linien darstellen
    line_pixels = cv2.countNonZero(edges)

    return line_pixels

# Hauptlogik für die Steuerung
line_counter = 0  # Zähler für erkannte Linien

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array  # Das aktuelle Bild aus dem Buffer holen

    # Linienerkennung durchführen
    line_pixels = line_detection(image)

    # Wenn genügend Linienpixel erkannt werden, erhöhen wir den Zähler
    if line_pixels > 500:  # Schwellenwert, um eine Linie zu erkennen
        line_counter += 1
        print(f"Linie erkannt! Gesamtzahl: {line_counter}")

    # Wenn der Roboter 24 Linien erkannt hat, stoppe ihn
    if line_counter >= 24:
        print("24 Linien erkannt. Roboter stoppt.")
        stop()  # Roboter anhalten
        break  

    # Wenn weniger als 8 Linien erkannt werden, fährt der Roboter weiter
    move_forward()

    # Buffer löschen
    rawCapture.truncate(0)

    # Wenn die Taste 'q' gedrückt wird, das Programm beenden
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# GPIO sauber beenden
GPIO.cleanup()
cv2.destroyAllWindows()
