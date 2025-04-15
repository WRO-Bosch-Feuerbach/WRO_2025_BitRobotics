import time
import cv2
import numpy as np
from picamera2 import Picamera2
import Lenkung

# === INITIALISIERUNG ===

camera = Picamera2 ()
camera.preview_configuration.main.size = (250, 140)
camera.framerate = 32
camera.preview_configuration.main.format = "RGB888"
camera.configure("preview")
camera.start()

# === HSV-Farbbereiche ===
# Für Hindernisse (RAL-Tne angenähert)

rot_min1= np.array([125, 100, 100])
rot_max1 = np.array ( [140, 255, 255])
rot_min2 = np.array ( [160, 100, 100])
rot_max2 = np.array ( [180, 255, 255])
gruen_min = np.array ( [50, 100, 100])
gruen_max = np.array ( [80, 255, 255])

pixel_threshold = 100

def erkenne_hindernis_farbe () :
    frame = camera.capture_array()
    hsv = cv2.cvtColor (frame, cv2.COLOR_RGB2HSV)
    mask_rot1 = cv2.inRange(hsv, rot_min1, rot_max1)
    mask_rot2 = cv2.inRange(hsv, rot_min2, rot_max2)
    mask_rot = cv2.bitwise_or(mask_rot1, mask_rot2)
    mask_gruen = cv2.inRange(hsv, gruen_min, gruen_max)
    red_count = cv2.countNonZero(mask_rot)
    green_count = cv2.countNonZero(mask_gruen)
    
    abweichung = 0
    if red_count > pixel_threshold:
        Farbe = 'ROT'
        print("rot")
        Lenkung.LenkungRechts()
        abweichung += 1

    elif green_count > pixel_threshold:
        Farbe = 'GRUEN'
        print("gruen")
        Lenkung.LenkungLinks()
        abweichung -= 1

    else:
        Farbe= 'BitRobotics'
        print("BitRobotics")
        if abweichung > 0:
            for i in range(abweichung):
                Lenkung.LenkungLinks()
                time.sleep(0.5)
        elif abweichung < 0:
            for i in range(abs(abweichung)):
                Lenkung.LenkungRechts()
                time.sleep(0.5)
        abweichung = 0
        Lenkung.LenkungGerade()

    time.sleep(0.5)

while True:
    erkenne_hindernis_farbe()