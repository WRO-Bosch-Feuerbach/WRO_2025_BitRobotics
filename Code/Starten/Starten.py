
import RPi.GPIO as GPIO
import time
import subprocess

# Pin für den Piezo-Buzzer
BUZZER_PIN = 18

# Noten als Frequenzen (Hz)
notes = {
    "A4": 440.00, "B4": 493.88, "C5": 523.25, "G4": 392.00, "F#4": 370.00,
    "E4": 329.63, "D5": 587.33
}

# Melodie (Noten und Dauer in Sekunden)
melody = [
    # Erster Takt
    ("A4", 0.33), ("B4", 0.33), ("C5", 0.67),
    ("B4", 0.33), ("A4", 0.33), ("G4", 0.67),
    ("F#4", 0.33), ("G4", 0.33),
]

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)#lll

# PWM initialisieren
buzzer = GPIO.PWM(BUZZER_PIN, 440)  # Start mit 440Hz
buzzer.start(50)  # 50% Duty Cycle

try:
    for note, duration in melody:
        if note in notes:
            buzzer.ChangeFrequency(notes[note])
            time.sleep(duration)
        time.sleep(0.05)  # Kleine Pause zwischen den Noten

    buzzer.stop()
    GPIO.cleanup()

except KeyboardInterrupt:
    buzzer.stop()
    GPIO.cleanup()


BUTTON_PIN = 5

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

lenkung_laeuft = None
antrieb_laeuft = None

def button_callback(channel):
    global antrieb_laeuft
    global lenkung_laeuft
    if lenkung_laeuft is None and antrieb_laeuft is None:
        print("Programm startet")
        antrieb_laeuft = subprocess.Popen(["python3", "Antrieb.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        lenkung_laeuft = subprocess.Popen(["python3", "Lenkung.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    else:
        print("Programm stoppen")
        if antrieb_laeuft:
            antrieb_laeuft.terminate()
        if lenkung_laeuft:
            lenkung_laeuft.terminate()
        
        lenkung_laeuft = None
        antrieb_laeuft = None
        GPIO.cleanup()

GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

try:
    print("Starten")
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Beendet")
    GPIO.cleanup()
    if lenkung_laeuft and antrieb_laeuft:
        lenkung_laeuft.terminate()
        antrieb_laeuft.terminate()