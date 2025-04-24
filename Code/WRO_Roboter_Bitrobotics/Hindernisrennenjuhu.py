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
import numpy as np

#----------------------------------------------
# Hinderniserkennung 
#----------------------------------------------

rot_min1 = np.array([125, 100, 100])
rot_max1 = np.array([140, 255, 255])
rot_min2 = np.array([160, 100, 100])
rot_max2 = np.array([180, 255, 255])
gruen_min = np.array([50, 100, 100])
gruen_max = np.array([80, 255, 255])

pixel_threshold = 15

abweichung = 0
last_hindernis_time = time.time()  # Initialisierung der last_hindernis_time
red_count = 0
green_count = 0

def erkenne_hindernis_farbe(frame):
    global abweichung, last_hindernis_time, red_count, green_count

    current_time = time.time()
    if current_time - last_hindernis_time < 0.2:  # Verhindert schnelle Richtungswechsel
        return

    last_hindernis_time = current_time
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask_rot1 = cv2.inRange(hsv, rot_min1, rot_max1)
    mask_rot2 = cv2.inRange(hsv, rot_min2, rot_max2)
    mask_rot = cv2.bitwise_or(mask_rot1, mask_rot2)
    mask_gruen = cv2.inRange(hsv, gruen_min, gruen_max)

    red_count = cv2.countNonZero(mask_rot)
    green_count = cv2.countNonZero(mask_gruen)


# ---------------------------------------------
# Linienerkennung als Hintergrund-Thread
# ---------------------------------------------

stop_robot_flag = False  # wird True, wenn 16 Linien erkannt wurden

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

        # Hinderniserkennung in jedem Frame
        erkenne_hindernis_farbe(image)

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
GPIO.setmode(GPIO.BCM)  # Pin-Nummerierung festlegen
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
    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_left(distanceR if distanceR <= 35 else distance))

def LenkungRechts():
    distanceL = Antrieb.LinksDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_right(distanceL if distanceL <= 35 else distance))

def LenkungGerade():
    winkel(0, 90)

def KopfdrehungVoraus():
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    winkel(2, 60)

# ---------------------------------------------
# Hauptprogrammstart
# ---------------------------------------------
if __name__ == '__main__':
    KopfneigungMitte()
    speed_set = 30
    Richtung = None
    counter = 0
    distance = Antrieb.checkDist()
    Antrieb.Motor(2, 1, speed_set)
    # Starte Linienerkennung und Hinderniserkennung parallel

    distanceL = Antrieb.LinksDist()
    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()

    linien_thread = threading.Thread(target=linien_erkennung, daemon=True)
    linien_thread.start()
    hindernis_thread = threading.Thread(target=erkenne_hindernis_farbe, daemon=True)
    hindernis_thread.start()

    if red_count > pixel_threshold:
        print("ROT erkannt")
        speed_set = 20
        Antrieb.Motor(2, 1, speed_set)
        LenkungRechts()
        if distanceR < 10:
            LenkungLinks()

    elif green_count > pixel_threshold:
        print("GRÜN erkannt")
        speed_set = 20
        Antrieb.Motor(2, 1, speed_set)
        LenkungLinks()
        if distanceL < 10:
            LenkungRechts()

    else:

        if counter == 0 and Richtung == None:  # Nur beim ersten Starten
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

    try:
        while not stop_robot_flag:
            distanceL = Antrieb.LinksDist()
            distanceR = Antrieb.RechtsDist()
            distance = Antrieb.checkDist()

            if None in (distance, distanceL, distanceR):
                print("Sensorfehler erkannt – überspringe Zyklus")
                continue

            # ======= 1. Farb-Hinderniserkennung (höchste Priorität) =======
            if red_count > pixel_threshold:
                print("ROT erkannt → Hindernis rechts")
                LenkungRechts()
                if distanceR < 5:
                    LenkungLinks()

            elif green_count > pixel_threshold:
                print("GRÜN erkannt → Hindernis links")
                LenkungLinks()
                if distanceL < 5:
                    LenkungRechts()

            # ======= 2. Kein Farb-Hindernis → klassische Abstandskontrolle =======
            elif distance <= 20:
                print("Sehr nahes Hindernis! Starte Ausweichmanöver...")
                Antrieb.motorStop()
                time.sleep(0.5)

                # Rückwärts & Entgegenlenken
                LenkungGerade()
                Antrieb.Motor(2, -1, speed_set)
                time.sleep(1.5)
                Antrieb.motorStop()

                if Richtung == "Rechts":
                    if distanceR < distanceL:
                        print("Innenkurve erkannt. Lenke nach Links zum Ausweichen.")
                        LenkungLinks()
                    else:
                        print("Außenkurve erkannt. Lenke nach Rechts zum Ausweichen.")
                        LenkungRechts()

                elif Richtung == "Links":
                    if distanceL < distanceR:
                        print("Innenkurve erkannt. Lenke nach Rechts zum Ausweichen.")
                        LenkungRechts()
                    else:
                        print("Außenkurve erkannt. Lenke nach Links zum Ausweichen.")
                        LenkungLinks()

                Antrieb.Motor(2, 1, speed_set)
                time.sleep(2)
                LenkungGerade()
                Antrieb.motorStop()
                time.sleep(1)

            # ======= 3. Normales Navigieren über Distanzsensoren =======
            else:
                if distanceL <= 35:
                    LenkungRechts()
                elif distanceR <= 35:
                    LenkungLinks()
                else:
                    if distance <= 90:
                        if Richtung == "Links":
                            LenkungLinks()
                        elif Richtung == "Rechts":
                            LenkungRechts()
                        else:
                            LenkungGerade()
                    else:
                        if abs(distanceL - distanceR) > 5:
                            if distanceL < distanceR:
                                print("Roboter nicht mittig, fahre Rechts")
                                LenkungRechts()
                            elif distanceR < distanceL:
                                print("Roboter nicht mittig, fahre Links")
                                LenkungLinks()
                        else:
                            LenkungGerade()

            # ======= 4. Bewegung & Kopfhaltung =======
            Antrieb.Motor(2, 1, speed_set)
            KopfneigungMitte()
            KopfdrehungVoraus()

    except KeyboardInterrupt:
        print("Manuell abgebrochen.")
    finally:
        print("Aufraeumen...")
        Antrieb.motorStop()
        KopfdrehungVoraus()
        KopfneigungMitte()
        LenkungGerade()
        pca.deinit()
        GPIO.cleanup()
