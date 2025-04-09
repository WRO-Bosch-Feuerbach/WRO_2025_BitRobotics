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

# Definiere Motoren/Pins 
LEFT_MOTOR_FORWARD = ?
LEFT_MOTOR_BACKWARD = ?
RIGHT_MOTOR_FORWARD = ?
RIGHT_MOTOR_BACKWARD = ?

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

# Farberkennung und Entscheidungslogik
def ColorDetection(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Farben definieren (HSV-Bereich für Grün und Rot)
    green_lower = (35, 40, 40)
    green_upper = (85, 255, 255)

    red_lower = (0, 100, 100)
    red_upper = (10, 255, 255)

    # Erkenne grünes Hindernis
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)
    red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)

    green_count = cv2.countNonZero(green_mask)
    red_count = cv2.countNonZero(red_mask)

    if green_count > 500:
        return "GREEN"
    elif red_count > 500:
        return "RED"
    else:
        return "NO_OBSTACLE"

# Hauptlogik für die Steuerung
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array  # Das aktuelle Bild aus dem Buffer holen
    detected_color = ColorDetection(image)  # Farberkennung durchführen

    if detected_color == "GREEN":
        print("Grünes Hindernis erkannt. Wende nach links!")
        move_backward()  # Rückwärts fahren
        sleep(1)  # Rückwärts fahren für 1 Sekunde
        turn_left()  # Nach links drehen
        sleep(1)  # 1 Sekunde lang nach links drehen
        move_forward()  # Weiterfahren
        print("Weiterfahren...")
    elif detected_color == "RED":
        print("Rotes Hindernis erkannt. Wende nach rechts!")
        move_backward()  
        sleep(1)  
        turn_right()  
        sleep(1)  
        move_forward()
        print("Weiterfahren...")
    else:
        print("Kein Hindernis erkannt. Weiterfahren...")
        move_forward()  

    # Buffer löschen
    rawCapture.truncate(0)

    # Wenn die Taste 'q' gedrückt wird, das Programm beenden
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# GPIO sauber beenden
GPIO.cleanup()
cv2.destroyAllWindows()
