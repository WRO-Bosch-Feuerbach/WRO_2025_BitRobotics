import time
from turtle import width
import cv2

cap = cv2.VideoCapture(0) #öffnet Kamera 0 oder 1
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 250)  #Auflösung in Pixel
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 140)

def ColorDetection():

	OrangeSection = 0
	BlueSection = 0

	#while True:
	_, frame = cap.read()  #liest aktuelle Bild aus dem Kamerafeed

	if frame is None:
		print("Bild konnte nicht geladen werden")  #Bild nicht geladen
		exit()

	hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  #BGR zu HSV(Farbton, Sättigung, Helligkeit)

	color = "nicht definiert"
	height, width, _ = frame.shape

	#cx = int(width / 2)
	#cy = int(height / 2)

	AreaStartPixelX = int(width/2) -10  #definiert den Bereich wo beachtet wird, also Rechteck
	AreaStartPixelY = int(height/2) - 20
	AreaEndPixelX = int(width/2) +5
	AreaEndPixelY = int(height/2) +5

	roi = hsv_frame[AreaStartPixelY:AreaEndPixelY, AreaStartPixelX:AreaEndPixelX]  #roi (Region of Interest) 

	#pixel_Area = hsv_frame[roi]
	
	pick_Farbton_Helligkeit = roi[:, :, 0]  #Farbton (Hue) des ausgewählten Bereichs wird extrahiert

#	print(picked_hue_value)
	
	for Farbton_Helligkeit in pick_Farbton_Helligkeit.flatten():  #jeder Farbton wird einzeln durchgegangen
#		print(hue_value)
		if 9 < Farbton_Helligkeit < 13:
			OrangeSection = OrangeSection + 1
#			print(HueValueIsOrange)
		elif 105 < Farbton_Helligkeit < 115:  #wie viele Pixel in den Bereichen ist
			BlueSection = BlueSection + 1
#			print(HueValueIsBlue)

	if OrangeSection >= 10:  #wenn 10 oder mehr pixel erkannt, dann dieser farbton
		color = "ORANGE"
		OrangeSection = 0
		print(color)
		return color
	elif BlueSection >= 10:
		color = "BLUE"
		BlueSection = 0
		print(color)
		return color
	else:
		color = "WHITE"
		print(color)
		return color

while True:
    color = ColorDetection()     #läuft kontinuierlich,ruft ColorDetection()-Funktion auf. 
    if cv2.waitKey(1) & 0xFF == ord('q'):  #wartet auf Tasteneingabe "q", dann Loop wird beendet.
        break

cap.release()   #beendet Kameraaufnahme und schließt alle OpenCV-Fenster
cv2.destroyAllWindows()







	


		#highlightedArea = frame.copy()

		#highlightedArea[hue_mask] = [0, 0, 255]
		#print(pixel_center)
		#cv2.putText(frame, color, (10, 50), 0, 1, (255, 0, 0), 2)
		#cv2.circle(frame, (cx, cy), 5, (255, 0, 0), 3)

		#cv2.imshow("Frame", frame)
		#time.sleep(0.5)
		#key = cv2.waitKey(1)
		#if key == 27:
			#break

#cap.release()
#cv2.destroyAllWindows()
