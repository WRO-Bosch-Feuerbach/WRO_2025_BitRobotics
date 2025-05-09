import RPi.GPIO as GPIO
import time
import subprocess
import Antrieb  # fürs Stoppen der Motoren

BUTTON_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

lenkung_laeuft = None


def button_callback(channel):
    global lenkung_laeuft

    if lenkung_laeuft is None:
        print(" Starte Fahrprogramm...")
        lenkung_laeuft = subprocess.Popen(["/usr/bin/python3","/home/bitrobotics/Desktop/Hindernisrennenjuhu.py" ])     
    else:
      print(" Stoppe Fahrprogramm...")
      Antrieb.motorStop()
      GPIO.cleanup()
      lenkung_laeuft.terminate()
      lenkung_laeuft = None
       

GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)

# ---------- MAIN LOOP ----------
try:
    print(" System bereit – drücke den Knopf zum Starten")
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print(" Manuelles Beenden – Aufräumen...")
    GPIO.cleanup()
    if lenkung_laeuft:
        lenkung_laeuft.terminate()
        

