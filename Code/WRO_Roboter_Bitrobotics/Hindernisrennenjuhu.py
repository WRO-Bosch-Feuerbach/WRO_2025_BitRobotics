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
import queue

#----------------------------------------------
# Hinderniserkennung 
#----------------------------------------------

rot_min1 = np.array([125, 100, 100])
rot_max1 = np.array([140, 255, 255])
rot_min2 = np.array([160, 100, 100])
rot_max2 = np.array([180, 255, 255])
gruen_min = np.array([50, 100, 100])
gruen_max = np.array([80, 255, 255])

pixel_threshold = 10

abweichung = 0
last_hindernis_time = time.time()  # Initialisierung der last_hindernis_time
red_count = 0
green_count = 0

# Queue zur Synchronisation der Bilddaten
image_queue = queue.Queue()

camera = PiCamera()
camera.resolution = (256, 144)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(256, 144))

print("Kamera fährt hoch...")
time.sleep(5)

def capture_images():
    """ Dieser Thread erfasst kontinuierlich Bilder von der Kamera. """
    global rawCapture
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        # Bild in die Queue einfügen
        image_queue.put(image)
        rawCapture.truncate(0)  # Buffer löschen

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

    return(red_count, green_count)

def linien_erkennung():
    global stop_robot_flag, Richtung

    line_counter = 0
    last_detected_color = None
    last_line_time = 0
    debounce_time = 1.0  # Sekunden, Entprellungszeit
    MAX_LINES = 24

    print("Linienerkennung gestartet...")

    while True:
        if stop_robot_flag:
            break

        # Hole das aktuelle Bild aus der Queue
        if not image_queue.empty():
            image = image_queue.get()
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
    winkel(2, 40)

KopfneigungMitte()

# ---------------------------------------------
# Hauptprogrammstart
# ---------------------------------------------
if __name__ == '__main__':

    # ---------------------------------------------
    # Linienerkennung als Hintergrund-Thread
    # ---------------------------------------------

    hindernis_thread = threading.Thread(target=erkenne_hindernis_farbe, daemon=True)
    hindernis_thread.start()

    linien_thread = threading.Thread(target=linien_erkennung, daemon=True)
    linien_thread.start()

    # ---------------------------------------------
    # Capture-Thread starten
    # ---------------------------------------------
    capture_thread = threading.Thread(target=capture_images, daemon=True)
    capture_thread.start()

    KopfneigungMitte()
    speed_set = 30
    Richtung = None
    counter = 0
    distance = Antrieb.checkDist()

    print(f"Es wird so viel ROT erkannt: {red_count}")
    print(f"Es wird so viel GRÜN erkannt: {green_count}")
    time.sleep(5)

    Antrieb.Motor(2, 1, speed_set)
    # Starte Linienerkennung und Hinderniserkennung parallel

    distanceL = Antrieb.LinksDist()
    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()

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
            while True:
                distance = Antrieb.checkDist()  # Abstand messen

                if distance <= 90:  # Zielabstand
                    Antrieb.motorStop()
                    print("Anhaltepunkt erreicht, Roboter stoppt.")
                    counter = 1
                    break  # Beende die Schleife und gehe zur nächsten Phase der Linienerkennung
                else:
                    Antrieb.Motor(2, 1, speed_set)

        while Richtung is None and not stop_robot_flag:
            time.sleep(0.1)

    try:
        while not stop_robot_flag:
            distanceL = Antrieb.LinksDist()
            distanceR = Antrieb.RechtsDist()
            distance = Antrieb.checkDist()

            if None in (distance, distanceL, distanceR):
                print("Sensorfehler erkannt – überspringe Zyklus")
                continue

            # ======= 1. Farb-Hinderniserkennung =======
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
            else:
                if distance <= 90:  # Zielabstand
                    Antrieb.motorStop()
                    print("Anhaltepunkt erreicht, Roboter stoppt.")
                    stop_robot_flag = True

                else:
                    Antrieb.Motor(2, 1, speed_set)

    except KeyboardInterrupt:
        print("Programm beendet.")
