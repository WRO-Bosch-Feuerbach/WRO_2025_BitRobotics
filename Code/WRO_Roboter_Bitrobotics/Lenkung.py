from locale import ERA
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



ER = 15 # Echo R
TR = 14 # Trigger R
TL = 22 # Trigger L
EL = 27 # Echo L
TF = 23 # Trigger vorne
EF = 24 # Echo vorne
Linkssensor = DistanceSensor(echo=EL, trigger=TL, max_distance = 2)
Rechtssensor = DistanceSensor(echo = ER,trigger = TR, max_distance = 2 )
sensor = DistanceSensor(echo=EF, trigger=TF, max_distance = 2)

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min
    
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
    

def LenkungLinks():
    #print("Ich Lenke Links")
    if distanceR < 15 or distance < 15:
        winkel(0, 120)
    else:
        winkel(0, max(0, min(180, map(distance, 75, 110, 120, 90))))

def LenkungRechts():
    #print("Ich Lenke Rechts")
    if distanceL < 15 or distance < 15:
        winkel(0, 60)
    else:
        winkel(0, max(0, min(180, map(distance, 75, 110, 60, 90))))

def LenkungGerade():
    #print("Ich Lenke Gerade")
    winkel(0, 90)

def KopfdrehungVoraus():
    #print("Ich schaue Voraus")
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    #print("Ich schaue geradeaus")
    winkel(2, 60)
    
def checkDist():
    return(sensor.distance) * 100 #unit in cm
    
def LinksDist():
    return(Linkssensor.distance) * 100

def RechtsDist():
    return(Rechtssensor.distance) * 100

distanceR = RechtsDist()
distance = checkDist()
distanceL = LinksDist()

if __name__ == '__main__':

    while distance >= 6:
        try:
            
            while True: #Funktioniert, wechselt aber noch zwischen Links/rechts schauen

                KopfneigungMitte()
                distanceR = RechtsDist()
                distance = checkDist()
                distanceL = LinksDist()
                KopfdrehungVoraus()

                if distance <= 50:
                    speed_set = 25
                else:
                    speed_set = 30
                

                if distance <= 40 or distanceL < 30 or distanceR < 30:
                    distanceL = LinksDist()
                    distanceR = RechtsDist()
                    if distanceL < distanceR:
                        LenkungRechts()
                        time.sleep(1)
                    elif distanceR < distanceL:
                        LenkungLinks()
                        time.sleep(1)
                    else:
                        LenkungGerade()

                Antrieb.Motor(2, 1, speed_set)
        
        except KeyboardInterrupt:
            Antrieb.motorStop()
            KopfdrehungVoraus()
            KopfneigungMitte()
            LenkungGerade()
            pca.deinit()
            exit()
