import RPi.GPIO as GPIO
import time

# ---------- GPIO Setup ----------
GPIO.setmode(GPIO.BCM)

# ---------- BUZZER ----------
BUZZER_PIN = 18
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Noten als Frequenzen (Hz)
notes = {
    "C5": 523.25,
    "D5": 587.33,
    "E5": 659.25,
    "F5": 698.46,
    "G5": 783.99,
    "A5": 880.00,
    "C6": 1046.50,
}

# Melodie um zu hören wann das PiCar hochgefahren ist (verkürzte Noten und Dauer in Sekunden)
melody = [
    ("C5", 0.4), ("E5", 0.4), ("G5", 0.4),
    ("C6", 0.4), ("G5", 0.4), ("E5", 0.4), ("C5", 0.4),
]

# PWM initialisieren
buzzer = GPIO.PWM(BUZZER_PIN, 440)
buzzer.start(50)

# Main Loop
# Melodie abspielen
try:
    for note, duration in melody:
        if note in notes:
            buzzer.ChangeFrequency(notes[note])
            time.sleep(duration)
        time.sleep(0.05)
finally:
    buzzer.stop()