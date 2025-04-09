import time
from turtle import speed
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
from gpiozero import DistanceSensor
from time import sleep

Rx = 15
Tx = 14
Tr = 23
Ec = 24
Rücksensor = DistanceSensor(echo=Tx, trigger=Rx, max_distance = 2)
sensor = DistanceSensor(echo=Ec, trigger=Tr, max_distance = 2)

MOTOR_M1_IN1 = 15
MOTOR_M1_IN2 = 14
MOTOR_M2_IN1 = 13
MOTOR_M2_IN2 = 12
MOTOR_M3_IN1 = 11
MOTOR_M3_IN2 = 10
MOTOR_M4_IN1 = 8
MOTOR_M4_IN2 = 9

Dir_forward = 0
Dir_backward = 1

left_forward = 1
left_backward = 0

right_forward = 0
right_backward = 1

pwn_A = 0
pwm_B = 0

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min)/(in_max - in_min)*(out_max - out_min) + out_min

i2c = busio.I2C(SCL, SDA)

pwm_motor = PCA9685(i2c, address = 0x5f) # = Motor
pwm_motor.frequency = 1000
pca = PCA9685(i2c, address = 0x5f) # PCA = Servo
pca.frequency = 50

motor1 = motor.DCMotor(pwm_motor.channels[MOTOR_M1_IN1], pwm_motor.channels[MOTOR_M1_IN2])
motor1.decay_mode = (motor.SLOW_DECAY)
motor2 = motor.DCMotor(pwm_motor.channels[MOTOR_M2_IN1], pwm_motor.channels[MOTOR_M2_IN2])
motor2.decay_mode = (motor.SLOW_DECAY)
motor3 = motor.DCMotor(pwm_motor.channels[MOTOR_M3_IN1], pwm_motor.channels[MOTOR_M3_IN2])
motor3.decay_mode = (motor.SLOW_DECAY)
motor4 = motor.DCMotor(pwm_motor.channels[MOTOR_M4_IN1], pwm_motor.channels[MOTOR_M4_IN2])
motor4.decay_mode = (motor.SLOW_DECAY)


def Motor(channel, direction, motor_speed):
    if motor_speed > 100:
        motor_speed = 100
    elif motor_speed < 0:
        motor_speed = 0
    speed = map(motor_speed, 0, 100, 0, 1.0)
    if direction == -1:
        speed = -speed

    if channel == 1:
        motor1.throttle = speed
    if channel == 2:
        motor2.throttle = speed
    if channel == 3:
        motor3.throttle = speed
    if channel == 4:
        motor4.throttle = speed

def checkDist():
    return(sensor.distance) * 100 #unit in cm
    
def rückDist():
    return(Rücksensor.distance) * 100

def motorStop():
    motor1.throttle = 0
    motor2.throttle = 0
    motor3.throttle = 0
    motor4.throttle = 0

def destroy():
    motorStop()
    pwm_motor.deinit()
    pca.deinit()
rückdistance = rückDist()
distance = checkDist()

if __name__ == '__main__':
    while distance >= 6:
        try:
            distance = checkDist()
            while distance >= 1:
                if distance > 90   :
                    speed_set = 70
                    Motor(1, 1, speed_set)
                    Motor(2, 1, speed_set)
                    Motor(3, 1, speed_set)
                    Motor(4, 1, speed_set)
                    distance = checkDist()
                    rückdistance = rückDist()
                    print("%2.f cm" %distance)


                elif distance < 90 and distance > 6:  
                    #print("Obstacle detected")
                    rückdistance = rückDist()
                    if rückdistance < 15:
                        speed_set = 0
                    else:
                        speed_set = 50
                    
                    Motor(1, 1, speed_set)
                    Motor(2, 1, speed_set)
                    Motor(3, 1, speed_set)
                    Motor(4, 1, speed_set)
                    distance = checkDist()
                    print("%2.f cm" %distance)
              
                if distance <= 6:
                    motorStop()
                    distance = checkDist()
                    time.sleep(2)


        except KeyboardInterrupt:
            motorStop()    
            destroy()
