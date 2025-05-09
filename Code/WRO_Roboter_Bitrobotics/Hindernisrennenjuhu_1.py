#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from tkinter import Y
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
import os
import sys

#----------------------------------------------
# Hinderniserkennung 
#----------------------------------------------

rot_min1 = np.array([125, 100, 100])
rot_max1 = np.array([140, 255, 255])
rot_min2 = np.array([160, 100, 100])
rot_max2 = np.array([180, 255, 255])
gruen_min = np.array([50, 70, 30])
gruen_max = np.array([80, 100, 80])
aktuelle_aktion = "Nichts"
pixel_threshold = 50

abweichung = 0
last_hindernis_time = time.time()  # Initialisierung der last_hindernis_time
red_count = 0
green_count = 0
counting = 0 # Variable gegen wiederholtes Rückwärtsfahren nach farberkennung, soll 0 sein


camera = PiCamera()
camera.resolution = (360, 640)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(360, 640))
print("Kamera Fährt hoch...")
time.sleep(5)

def zeige_status(distance, distanceL, distanceR, red_count, green_count, aktion, line_counter, MAX_LINES, Richtung, letzte_farbe, letzte_aktion):
    status_lines = [
        "=== ROBOT STATUS ===",
        f"Distanz Vorne:  {distance:>5} cm",
        f"Distanz Links:  {distanceL:>5} cm",
        f"Distanz Rechts: {distanceR:>5} cm",
        f"Rote Pixel:     {red_count:>5}",
        f"Grüne Pixel:    {green_count:>5}",
        f"Aktuelle Aktion: {aktion}",
        f"Letzte Aktion: {letzte_aktion}",
        f"Anzahl Linien: {line_counter}/{MAX_LINES}",
        f"Letzte Farbe: {letzte_farbe}",
        f"Richtung: {Richtung}",
        "===================="
    ]
    counter = 0
    if counter == 0:
        os.system('clear')
        counter = 1

    # Cursor nach oben bewegen, um vorige Ausgabe zu überschreiben
    sys.stdout.write('\033[F' * len(status_lines))
    for line in status_lines:
        print(line)
    
def erkenne_hindernis_farbe(frame):
    global abweichung, last_hindernis_time, red_count, green_count

    current_time = time.time()
    if current_time - last_hindernis_time < 0.05:  # Verhindert schnelle Richtungswechsel
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
    global stop_robot_flag, Richtung, line_counter, MAX_LINES

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

    def is_curuve_ahead(distanceL, distanceR):
        #Frühe Kurvenerkennung
        return abs(distanceL - distanceR) > 10

# ---------------------------------------------
# Originales Steuerungsskript (leicht angepasst)
# ---------------------------------------------

# I2C & Servo Setup
GPIO.setmode(GPIO.BCM)  # Pin-Nummerierung festlegen
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

servos = [
    servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2400, actuation_range=180),  # Lenkung
    servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2400, actuation_range=180),  # Kopfdrehung
    servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2400, actuation_range=180)   # Kopfneigung
]
    
def winkel(ID, winkelwert):
    """Setze den Winkel des Servos an ID."""
    if 0 <= ID < len(servos):
        if winkelwert is None or not isinstance(winkelwert, (int, float)):
            return
        winkelwert = max(0, min(180, winkelwert))
        servos[ID].angle = winkelwert
 
def Kopfwinkel(ID, winkelwert):
    """Setze den Kopfservo auf einen bestimmten Winkel."""
    if 0 <= ID < len(servos):
        if winkelwert is None or not isinstance(winkelwert, (int, float)):
            return
        winkelwert = max(0, min(180, winkelwert))
        servos[ID].angle = winkelwert 

def distance_to_angle_left(distance):
    if distance is None:
        return 90
    return 90 + ((200 - min(200, distance)) / 200) * 40

def distance_to_angle_right(distance):
    if distance is None:
        return 90
    return 90 - ((200 - min(200, distance)) / 200) * 40

last_turn_direction = None

def LenkungLinks():
    global last_turn_direction
    aktuelle_aktion = "Lenke Links"

    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_left(distanceR if distanceR <= 35 else distance))
    last_turn_direction = "links"

def LenkungRechts():
    global last_turn_direction
    aktuelle_aktion = "Lenke Rechts"

    distanceL = Antrieb.LinksDist()
    distance = Antrieb.checkDist()
    winkel(0, distance_to_angle_right(distanceL if distanceL <= 35 else distance))
    last_turn_direction = "rechts"

def LenkungGerade():
    winkel(0, 90)

def KopfdrehungVoraus():
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    winkel(2, 35)

KopfneigungMitte()

# ---------------------------------------------
# Funktionen für die Kurven- und Mittigkeitslogik
# ---------------------------------------------


def is_in_turn():
    distanceL = Antrieb.LinksDist()
    distanceR = Antrieb.RechtsDist()
    return abs(distanceL - distanceR) > 10
    

# ---------------------------------------------
# Hauptprogrammstart
# ---------------------------------------------
if __name__ == '__main__':

    hindernis_thread = threading.Thread(target=erkenne_hindernis_farbe, daemon=True)
    hindernis_thread.start()
    linien_thread = threading.Thread(target=linien_erkennung, daemon=True)
    linien_thread.start()


    KopfneigungMitte()
    speed_set = 30
    Richtung = None
    counter = 0
    distance = Antrieb.checkDist()

    while Richtung == None:
        camera.capture(rawCapture, format="bgr")
        frame = rawCapture.array
    
        erkenne_hindernis_farbe(frame)

        print(f"Initial ROT: {red_count}")
        print(f"Initial GRÜN: {green_count}")

        Antrieb.Motor(2, 1, speed_set)

        rawCapture.truncate(0)
        distanceL = Antrieb.LinksDist()
        distanceR = Antrieb.RechtsDist()
        distance = Antrieb.checkDist()

    distanceL = Antrieb.LinksDist()
    distanceR = Antrieb.RechtsDist()
    distance = Antrieb.checkDist()
    
    if red_count > pixel_threshold:
        print("ROT erkannt")
        speed_set = 20
        Antrieb.Motor(2, 1, speed_set)
        LenkungRechts()
        letzte_farbe = "ROT"
        if letzte_farbe == "ROT":
            aktuelle_aktion = ("Erkenne Rot, Lenke Rechts")
            if red_count > pixel_threshold:
                LenkungRechts()
            elif distanceL < 20:
                aktuelle_aktion = "Letzte Farbe Rot, Links Hindernis, !!Muss Rechts!!"
                LenkungRechts()
            elif red_count < pixel_threshold:
                LenkungLinks()
            else:
                aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                LenkungGerade()

    elif green_count > pixel_threshold:
        print("GRÜN erkannt")
        speed_set = 20
        Antrieb.Motor(2, 1, speed_set)
        LenkungLinks()
        letzte_farbe = "GRUEN"
        if letzte_farbe == "GRUEN":
            aktuelle_aktion = ("Erkenne Grün, Lenke Links")
            if green_count > pixel_threshold:
                LenkungLinks()                        
            elif distanceR < 20:
                aktuelle_aktion = "Letzte Farbe Grün, Rechts Hindernis, !!Muss Links!!"
                LenkungLinks()
            elif green_count < pixel_threshold:
                LenkungRechts()
            else:
                aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                LenkungGerade()

    else:
        if counter == 0 and Richtung is None:
            speed_set = 30
            while True:
                distance = Antrieb.checkDist()
                if distance <= 50:
                    Antrieb.motorStop()
                    aktuelle_aktion = ("Anhaltepunkt erreicht, Roboter stoppt.")
                    counter = 1
                    break
                else:
                    Antrieb.Motor(2, 1, speed_set)

    try:
        letzte_farbe = None
        while not stop_robot_flag:
            distanceL = Antrieb.LinksDist()
            distanceR = Antrieb.RechtsDist()
            distance = Antrieb.checkDist()

            if None in (distance, distanceL, distanceR):
                print("Sensorfehler erkannt – überspringe Zyklus")
                continue

            # ======= 1. Farb-Hinderniserkennung =======
            if red_count > pixel_threshold:
                # Rückwärts fahren, für bessere umsicht
                aktuelle_aktion = ("ROT erkannt → Hindernis rechts")   
                if red_count > 8000 and counting == 0:
                    Antrieb.motorStop()
                    time.sleep(0.5)

                    LenkungGerade()
                    time.sleep(1)

                    Antrieb.Motor(2, -1, speed_set)
                    time.sleep(2)
                    counting = 1

                LenkungRechts()
                speed_set = 20
                Antrieb.Motor(2, 1, speed_set)
                letzte_farbe = "ROT"

                if letzte_farbe == "ROT":
                    aktuelle_aktion = ("Erkenne Rot, Lenke Rechts")
                    if red_count > pixel_threshold:
                        LenkungRechts()
                    elif distanceL < 20:
                        aktuelle_aktion = "Letzte Farbe Rot, Links Hindernis, !!Muss Rechts!!"
                        LenkungRechts()
                    elif red_count < pixel_threshold:
                        LenkungLinks()
                    else:
                        aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                        LenkungGerade()

            elif green_count > pixel_threshold:
                aktuelle_aktion =("GRÜN erkannt → Hindernis links")
                if green_count > 6000 and counting == 0:
                    Antrieb.motorStop()
                    time.sleep(0.5)

                    LenkungGerade()
                    time.sleep(1)

                    Antrieb.Motor(2, -1, speed_set)
                    time.sleep(2)
                    counting = 1

                LenkungLinks()
                speed_set = 20
                Antrieb.Motor(2, 1, speed_set)
                letzte_farbe = "GRUEN"

                if letzte_farbe == "GRUEN":
                    aktuelle_aktion = ("Erkenne Grün, Lenke Links")
                    if green_count > pixel_threshold:
                        LenkungLinks()                        
                    elif distanceR < 20:
                        aktuelle_aktion = "Letzte Farbe Grün, Rechts Hindernis, !!Muss Links!!"
                        LenkungLinks()
                    elif green_count < pixel_threshold:
                        LenkungRechts()
                    else:
                        aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                        LenkungGerade()

            # ======= 2. Klassische Abstandskontrolle =======
            if distance <= 25 and red_count < pixel_threshold and green_count < pixel_threshold:
                aktuelle_aktion = ("Sehr nahes Hindernis! Starte Ausweichmanöver...")
                Antrieb.motorStop()
                time.sleep(0.5)
                LenkungGerade()
                Antrieb.Motor(2, -1, speed_set)
                time.sleep(1.5)
                Antrieb.motorStop()

                if last_turn_direction == "rechts":
                    LenkungLinks()
                elif last_turn_direction == "links":
                    LenkungRechts()

                Antrieb.Motor(2, 1, speed_set)
                time.sleep(2)
                LenkungGerade()
                Antrieb.motorStop()
                time.sleep(1)

            # ======= 3. Normales Fahren =======
            else:

                if (red_count < pixel_threshold) and (green_count < pixel_threshold) and counting == 1:
                    letzte_farbe = None
                    counting = 0

                if letzte_farbe == "ROT":
                    aktuelle_aktion = ("Erkenne Rot, Lenke Rechts")
                    if red_count > pixel_threshold:
                        LenkungRechts()
                        time.sleep(0.3)
                        LenkungGerade()
                    elif distanceL < 20:
                        aktuelle_aktion = "Letzte Farbe Rot, Links Hindernis, !!Muss Rechts!!"
                        LenkungRechts()
                    elif red_count < pixel_threshold:
                        LenkungLinks()
                    else:
                        aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                        LenkungGerade()

                elif letzte_farbe == "GRUEN":
                    aktuelle_aktion = ("Erkenne Grün, Lenke Links")
                    if green_count > pixel_threshold:
                        LenkungLinks()                        
                    elif distanceR < 20:
                        aktuelle_aktion = "Letzte Farbe Grün, Rechts Hindernis, !!Muss Links!!"
                        LenkungLinks()
                    elif green_count < pixel_threshold:
                        LenkungRechts()
                    else:
                        aktuelle_aktion = ("Letzte Farbe Rot, alles gut!")
                        LenkungGerade()


                else:
                    
                    if distanceL <= 20 and not letzte_farbe == "GRUEN":
                        aktuelle_aktion = (" Zu nah links – korrigiere nach rechts")
                        LenkungRechts()

                    elif distanceR <= 20 and not letzte_farbe == "ROT":
                        aktuelle_aktion = (" Zu nah rechts – korrigiere nach links")
                        LenkungLinks()

                    else:
                        if distance <= 80:
                            if Richtung == "Links":
                                LenkungLinks()
                            elif Richtung == "Rechts":
                                LenkungRechts()
                            else:
                                LenkungGerade()
                        else:
                            LenkungGerade()
                                
            # ======= 4. Bewegung & Kopfhaltung =======
            Antrieb.Motor(2, 1, speed_set)
            KopfneigungMitte()
            KopfdrehungVoraus()
            if red_count < pixel_threshold and green_count < pixel_threshold:
                speed_set = 30

            letzte_aktion = aktuelle_aktion

            zeige_status(distance, distanceL, distanceR, red_count, green_count, aktuelle_aktion, line_counter, MAX_LINES, Richtung, letzte_farbe, letzte_aktion)
            #time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Manuell abgebrochen.")
    finally:
        print("Aufraeumen...")
        Antrieb.motorStop()
        KopfdrehungVoraus()
        KopfneigungMitte()
        LenkungGerade()
        pca.deinit()
        camera.close()
        GPIO.cleanup()
