from locale import ERA
from re import L
import time
from turtle import speed
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo
from gpiozero import DistanceSensor
import CameraColorDetection2 as Camera
import Antrieb
from time import sleep
import math


    
i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address = 0x5f) # PCA = Servo
pca.frequency = 50


def winkel(ID, winkel):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse = 500, max_pulse = 2400, actuation_range = 180)
    servo_angle.angle = winkel
    print(servo_angle.angle)
    return(servo_angle.angle)

def Kopfwinkel(ID, winkel):
    servo_angle = servo.Servo(pca.channels[ID], min_pulse = 500, max_pulse = 2400, actuation_range = 180)
    servo_angle.angle = winkel
    print(servo_angle.angle)
    return(servo_angle.angle)
   
def distance_to_angle_left(distance):
    distance = max(0, min(200, distance))
    winkel = 90 + ((200 - distance) / 200) * 60  # Hier anstelle 90° nur 60° maximal
    return winkel

def distance_to_angle_right(distance):
    distance = max(0, min(200, distance))
    winkel = 90 - ((200 - distance) / 200) * 60  # Hier anstelle 90° nur 60° maximal
    return winkel

def LenkungLinks():
    # Berechnung des Lenkwinkels basierend auf der Entfernung
    if distanceR <= 35:
        winkel_value = distance_to_angle_left(distanceR)
        winkel(0, winkel_value)  # Maximale Lenkung nach links
    else:
        winkel_value = distance_to_angle_left(distance)  # Dynamischer Winkel basierend auf der Distanz
        winkel(0, winkel_value)


def LenkungRechts():
    # Berechnung des Lenkwinkels basierend auf der Entfernung
    if distanceL <= 35:
        winkel_value = distance_to_angle_right(distanceL)
        winkel(0, winkel_value)  # Maximale Lenkung nach rechts
    else:
        winkel_value = distance_to_angle_right(distance)  # Dynamischer Winkel basierend auf der Distanz
        winkel(0, winkel_value)



def LenkungGerade():
    #print("Ich Lenke Gerade")
    winkel(0, 90)

def KopfdrehungVoraus():
    #print("Ich schaue Voraus")
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    #print("Ich schaue geradeaus")
    winkel(2, 60)
    

distanceR = Antrieb.RechtsDist()
distance = Antrieb.checkDist()
distanceL = Antrieb.LinksDist()

if __name__ == '__main__':
    count = 0
    Richtung = "Null"
    speed_set = 40
    while distance >= 6:
        try:
            while True: #Funktioniert, wechselt aber noch zwischen Links/rechts schauen


                Antrieb.Motor(2, 1, speed_set)
                if count == 0:
                    if distance <= 30:
                        Antrieb.motorStop()
                        time.sleep(2)
                        if distanceL < distanceR:
                            Richtung = "Rechts"
                            print(f"Rechts ist mehr Platz, Fahre {Richtung} Herum")
                            count = 1
                            Antrieb.Motor(2, -1, 30)
                            time.sleep(2)
                        elif distanceR < distanceL:
                            Richtung = "Links"
                            print(f"Links ist mehr Platz, Fahre {Richtung} Herum")
                            count = 1
                            Antrieb.Motor(2, -1, 30)
                            time.sleep(2)
                        else:
                            print("Keine Richtung erfasst.")
                            #time.sleep(5)

                distanceL = Antrieb.Linkssensor.distance * 100
                distanceR = Antrieb.Rechtssensor.distance * 100
                distance = Antrieb.sensor.distance * 100
                print(f"Links: {distanceL:.1f} cm | Rechts: {distanceR:.1f} cm | Vorne: {distance:.1f} cm")
  
                if distanceL <= 40:
                    LenkungRechts()
                elif distanceR <= 40:
                    LenkungLinks()
                else:
                    if distance <= 90 and Richtung == "Links":
                        LenkungLinks()
                    elif distance <= 90 and Richtung == "Rechts":
                        LenkungRechts()
                    else:
                        if abs(distanceL - distanceR) > 5:
                            if distanceL < distanceR:
                                print("Roboter nicht mittig, Fahre Rechts")
                                LenkungRechts()
                            elif distanceR < distanceL:
                                print("Roboter nicht mittig, Fahre Links")
                                LenkungLinks()
                            else:
                                LenkungGerade()
 
                # Geschwindigkeit basierend auf dem Frontabstand
                if distance <= 90:
                    speed_set = 30
                elif distance > 90:
                    speed_set = 40
                else:
                    speed_set = 30
 
                # Motoren steuern
                Antrieb.Motor(2, 1, speed_set)  # Vorwärts
                Antrieb.Motor(3, 1, speed_set)  # Vorwärts
                Antrieb.Motor(4, 1, speed_set)  # Vorwärts
 
                #time.sleep(0.1)

                KopfneigungMitte()
                distanceR = Antrieb.RechtsDist()
                distance = Antrieb.checkDist()
                distanceL = Antrieb.LinksDist()
                KopfdrehungVoraus()                

                Antrieb.Motor(2, 1, speed_set)
        
        except KeyboardInterrupt:
            Antrieb.motorStop()
            KopfdrehungVoraus()
            KopfneigungMitte()
            LenkungGerade()
            pca.deinit()
            exit()
