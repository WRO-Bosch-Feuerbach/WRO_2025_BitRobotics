#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import cv2
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import Antrieb
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray

# ---------------------------------------------
# Linienerkennung als Hintergrund-Thread
# ---------------------------------------------

stop_robot_flag = False  # Wird True, wenn 24 Linien erkannt wurden

def color_detection(frame, color_to_detect):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color_to_detect == "blue":
        lower = (105, 130, 50)
        upper = (125, 255, 255)
    elif color_to_detect == "orange":
        lower = (10, 150, 80)
        upper = (22, 255, 255)
    else:
        return 0

    mask = cv2.inRange(hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    line_like_objects = 0

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / h if h != 0 else 0
        area = cv2.contourArea(cnt)

        if area > 200 and (aspect_ratio > 2 or aspect_ratio < 0.5):
            line_like_objects += 1

    return line_like_objects

def linien_erkennung():
    global stop_robot_flag, Richtung

    camera = PiCamera()
    camera.resolution = (250, 140)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(250, 140))
    time.sleep(2)

    line_counter = 0
    last_detected_color = None
    last_line_time = 0
    debounce_time = 1.0  # Sekunden, Entprellungszeit
    MAX_LINES = 24

    print("Linienerkennung gestartet...")

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if stop_robot_flag:
            break

        image = frame.array
        height, width, _ = image.shape
        image_boden = image[int(height / 2):, :].copy()
        current_time = time.time()

        # Für die erste Linie keine Entprellung anwenden
        if line_counter == 0:
            # Suche nach blauer Linie
            count_blue = color_detection(image_boden, "blue")
            if count_blue > 0:
                last_detected_color = "blue"
                last_line_time = current_time
                Richtung = "Links"  # Erste Linie blau, Richtung ist links
                print("Erste Linie: Blau erkannt. Richtung ist linksherum")
                line_counter += 1
                print(f"[{line_counter}/{MAX_LINES}] Blaue Linie erkannt")

            # Suche nach orangener Linie
            count_orange = color_detection(image_boden, "orange")
            if count_orange > 0:
                last_detected_color = "orange"
                last_line_time = current_time
                Richtung = "Rechts"  # Erste Linie orange, Richtung ist rechts
                print("Erste Linie: Orange erkannt. Richtung ist rechtsherum")
                line_counter += 1
                print(f"[{line_counter}/{MAX_LINES}] Orange Linie erkannt")
        
        # Für alle nachfolgenden Linien den Entprellungscheck anwenden
        elif (current_time - last_line_time) > debounce_time:
            # Suche nach blauer Linie
            count_blue = color_detection(image_boden, "blue")
            if count_blue > 0 and last_detected_color != "blue":
                last_detected_color = "blue"
                last_line_time = current_time
                line_counter += 1
                print(f"[{line_counter}/{MAX_LINES}] Blaue Linie erkannt")

            # Suche nach orangener Linie
            count_orange = color_detection(image_boden, "orange")
            if count_orange > 0 and last_detected_color != "orange":
                last_detected_color = "orange"
                last_line_time = current_time
                line_counter += 1
                print(f"[{line_counter}/{MAX_LINES}] Orange Linie erkannt")

        # Wenn MAX_LINES erreicht sind, stoppen wir den Roboter
        if line_counter > MAX_LINES:
            print("Ziel erreicht: 24 Linien erkannt. Roboter wird gestoppt.")
            stop_robot_flag = True
            Antrieb.motorStop()
            break

        rawCapture.truncate(0)

    camera.close()
    print("Linienerkennung beendet.")

# ---------------------------------------------
# Originales Steuerungsskript (leicht angepasst)
# ---------------------------------------------

# I2C & Servo Setup
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

def winkel(ID, winkelwert):
    if winkelwert is None or not isinstance(winkelwert, (int, float)):
        return
    winkelwert = max(0, min(180, winkelwert))
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = winkelwert

def Kopfwinkel(ID, winkelwert):
    if winkelwert is None or not isinstance(winkelwert, (int, float)):
        return
    winkelwert = max(0, min(180, winkelwert))
    servo_angle = servo.Servo(pca.channels[ID], min_pulse=500, max_pulse=2400, actuation_range=180)
    servo_angle.angle = winkelwert

def distance_to_angle_left(distance):
    if distance is None:
        return 90
    return 90 + ((200 - min(200, distance)) / 200) * 40

def distance_to_angle_right(distance):
    if distance is None:
        return 90
    return 90 - ((200 - min(200, distance)) / 200) * 40

def LenkungLinks():
    distanceL = Antrieb.LinksDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_left(distanceL if distanceL <= 35 else distance))
    

def LenkungRechts():
    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_right(distanceR if distanceR <= 35 else distance))
    

def LenkungGerade():
    winkel(0, 90)  # Direkt in die Mitte lenken (90°)
    

def KopfdrehungVoraus():
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    winkel(2, 40)
KopfneigungMitte()
# ---------------------------------------------
# Funktionen für die Kurven- und Mittigkeitslogik
# ---------------------------------------------

# Mittigkeitsprüfung mit Toleranz von 10
def check_midline():
    distanceL = Antrieb.LinksDist()  # Abstand links
    distanceR = Antrieb.RechtsDist()  # Abstand rechts

    # Toleranz von 10 für die Mittigkeitsprüfung
    if abs(distanceL - distanceR) <= 10:
        print("Roboter ist mittig!")
        return True  # Roboter ist mittig
    else:
        print("Roboter ist nicht mittig!")
        return False  # Roboter ist nicht mittig

# Überprüfung, ob der Roboter sich in einer Kurve befindet
def is_in_turn():
    distanceL = Antrieb.LinksDist()
    distanceR = Antrieb.RechtsDist()

    # Eine Kurve wird angenommen, wenn der Unterschied zwischen den Distanzen > 15 ist
    return abs(distanceL - distanceR) > 10

# Steuerung, um den Roboter zu lenken, wenn er nicht mittig ist
def lenke_und_weiterfahren():
    """Überprüft, ob der Roboter mittig fährt, außer wenn er sich in einer Kurve befindet."""
    if is_in_turn():
        return False  # Wenn er sich in einer Kurve befindet, überspringe die Mittigkeitsprüfung
    return check_midline()  # Ansonsten immer Mittigkeit überprüfen

# ---------------------------------------------
# Hauptprogrammstart
# ---------------------------------------------
if __name__ == '__main__':
    speed_set = 35
    Richtung = None
    counter = 0
    distance = Antrieb.checkDist()

    # Starte Linienerkennung parallel
    linien_thread = threading.Thread(target=linien_erkennung, daemon=True)
    linien_thread.start()

    if counter == 0 and Richtung is None:  # Nur beim ersten Starten
        # Überwache kontinuierlich den Abstand bis die Linie erkannt wird
        while True:
            distance = Antrieb.checkDist()  # Abstand messen

            if distance <= 98:  # Zielabstand (näher an der Linie)
                Antrieb.motorStop()  # Roboter stoppen
                print("Anhaltepunkt erreicht, Roboter stoppt.")
                counter = 1  # Sicherstellen, dass der Code nur einmal ausgeführt wird
                break  # Beende die Schleife und gehe zur nächsten Phase der Linienerkennung
            else:
                # Weiterfahren, bis der Zielabstand erreicht ist
                Antrieb.Motor(2, 1, speed_set)  # Vorwärts fahren

    # Jetzt Linie erkennen und Richtung setzen
    while Richtung is None and not stop_robot_flag:
        time.sleep(0.1)  # Kurze Pause, um sicherzustellen, dass Linien erkannt werden können.

# ----------------------------- Anpassungen in der Hauptlogik -----------------------------
try:
    while not stop_robot_flag:
        distanceL = Antrieb.LinksDist()
        distanceR = Antrieb.RechtsDist()
        distance = Antrieb.checkDist()

        if None in (distance, distanceL, distanceR):
            print(" Sensorfehler erkannt – Roboter stoppt zur Sicherheit.")
            Antrieb.motorStop()
            time.sleep(0.2)
            continue

        if distance <= 20:
            print("‼️ Hindernis zu nah! Stoppe und weiche aus.")
            Antrieb.motorStop()
            time.sleep(0.5)

            # Rückwärts
            LenkungGerade()
            Antrieb.Motor(2, -1, speed_set)
            time.sleep(2)
            Antrieb.motorStop()

            # Neu messen und seitlich ausweichen
            distanceL = Antrieb.LinksDist()
            distanceR = Antrieb.RechtsDist()

            if Richtung == "Rechts":
                if distanceR < distanceL:
                    LenkungLinks()
                else:
                    LenkungRechts()
            elif Richtung == "Links":
                if distanceL < distanceR:
                    LenkungRechts()
                else:
                    LenkungLinks()

            Antrieb.Motor(2, 1, speed_set)
            time.sleep(2)
            LenkungGerade()
            Antrieb.motorStop()
            time.sleep(1)
            continue

        # Seiten-Korrekturen
        if distanceL <= 25:
            print(" Zu nah an der linken Wand – korrigiere nach rechts")
            LenkungRechts()
            time.sleep(0.3)
            LenkungGerade()

        elif distanceR <= 25:
            print(" Zu nah an der rechten Wand – korrigiere nach links")
            LenkungLinks()
            time.sleep(0.3)
            LenkungGerade()

        else:
            # Allgemeine Fahrlogik
            if distance <= 83:
                if Richtung == "Links":
                    LenkungLinks()
                elif Richtung == "Rechts":
                    LenkungRechts()
            else:
                if abs(distanceL - distanceR) > 15:
                    print("↪ In der Kurve – beibehalten.")
                    pass  # nichts tun
                else:
                    LenkungGerade()

        Antrieb.Motor(2, 1, speed_set)
        KopfneigungMitte()
        KopfdrehungVoraus()

except KeyboardInterrupt:
    print(" Programm manuell unterbrochen.")
    Antrieb.motorStop()

finally:
    print(" Aufräumen und Reset...")
    Antrieb.motorStop()
    KopfdrehungVoraus()
    KopfneigungMitte()
    LenkungGerade()
    pca.deinit()
    GPIO.cleanup()

