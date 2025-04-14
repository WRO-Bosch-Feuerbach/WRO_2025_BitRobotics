from locale import ERA
import time
from turtle import speed
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo
from gpiozero import DistanceSensor
from time import sleep



ER = 15 # Echo R
TR = 14 # Trigger R
EL = 22 # Echo L
TL = 27 # Trigger L
TF = 23 # Trigger vorne
EF = 24 # Echo vorne
Linkssensor = DistanceSensor(echo=TL, trigger=EL, max_distance = 2)
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
    if distance < 110:
        winkel(0, 120)
    else:
        winkel(0, max(0, min(180, map(distance, 75, 110, 120, 90))))

def LenkungRechts():
    #print("Ich Lenke Rechts")
    if distance < 110:
        winkel(0, 60)
    else:
        winkel(0, max(0, min(180, map(distance, 75, 110, 60, 90))))

def LenkungGerade():
    #print("Ich Lenke Gerade")
    winkel(0, 90)

def KopfdrehungLinks():
    #print("Ich schaue Links")
    Kopfwinkel(1, 140)

def KopfdrehungRechts():
    #print("Ich schaue Rechts")
    Kopfwinkel(1, 40)

def KopfdrehungVoraus():
    #print("Ich schaue Voraus")
    Kopfwinkel(1, 90)

def KopfneigungMitte():
    #print("Ich schaue geradeaus")
    winkel(2, 65)
    
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

                if distanceL <= 20:
                    LenkungRechts()
                elif distanceR <= 20:
                    LenkungLinks()
                else:
                    LenkungGerade()

#                distance = checkDist()
#                KopfdrehungLinks()
#                time.sleep(0.5)
#                if distance <= 20:
#                    LenkungRechts()
#                    distance = checkDist()
#                    print("%2.f cm" %distance)
#                else:
#                    LenkungGerade()
#                    print("%2.f cm" %distance)
#                time.sleep(1)
#                KopfdrehungRechts()
#                time.sleep(0.5)
#                if distance <= 20:
#                    LenkungLinks()
#                    distance = checkDist()
#                    print("%2.f cm" %distance)
#                else:
#                    LenkungGerade()
#                    print("%2.f cm" %distance)
#            
#            
#            KopfneigungMitte()
#            distance = checkDist()
#            while distance != 0:
#                if distance > 110:
#                    
#                    LenkungGerade()
#                    distance = checkDist()
#                    rückdistance = rückDist()
#                    print("%2.f cm" %distance)
#                    
#                elif distance < 110: 
#                    
#                    rückdistance = rückDist()
#                    distance = checkDist()
#                    print("%2.f cm" %distance)
#                    LenkungRechts()


        
        except KeyboardInterrupt:
            KopfdrehungVoraus()
            KopfneigungMitte()
            LenkungGerade()
            pca.deinit()
            exit()
