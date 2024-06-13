"""
Protocole de communication entre PC et moteurs:

Il existe 2 rotations : 
    -Yaw : Rotation selon l'axe z qui prend ses informations par rapport au son qui est traité
    -Pitch : Rotation selon l'axe y qui prend ses informations par rapport à l'image qui est traitée

Le pc renvera toujours un angle en degré et le traitement se fera directment sur la carte pour la conversion
Yaw [-180°;180°]
Pitch [-45°;45°]

On pourra avoir un protocole de communication tel que

YSXXX où Y signfie Yaw, S est soit + ou - pour montrer le signe et XXX est un entier qui represente un angle en degre

De façon analogique pour le Pitch on a 

PSXXX

exemple:

P+090 signifique que le robot fait une rotation de sur l'axe y de 90°
"""

import serial
import cv2
from tkinter import * 
from math import sqrt

def write_angle(serial_port: str ,rotation : str, signe : str, angle : str)->int:
    """
    Verification avant écriture si les informations sont bien de la forme PSXXX ou YSXXX
    return 1 si probleme
    return 0 si pas probleme
    """
    if type(rotation) is str and type(signe) is str and type(angle) is str:
        if rotation == "P" and rotation == "Y":
            if signe == "+" and signe == "-":
                if len(angle) == 3:
                    if int(angle) >= -180 and int(angle)<= 180:
                        serial_port.write(rotation+signe+angle)
                        return 0
    return 1

# configure the serial connections (the parameters differs on the device you are connecting to)
ser_motor = serial.Serial( # Port de connection à la carte de commande des moteurs
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser_motor.isOpen()

ser_mic = serial.Serial( # Port de connection à la carte de réception des micros
    port='/dev/ttyACM1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser_mic.isOpen()


################ Réconnaissance d'image ################
face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
video_capture = cv2.VideoCapture(0)


def is_centered(face_center, screen_center):
    if face_center[1] in [i for i in range(screen_center[1]-50,screen_center[1]+50)] and face_center[0] in [i for i in range(screen_center[0]-50,screen_center[0]+50)]:
        return True
    return False

def get_height_difference(face_center, screen_center):
    return screen_center[1]-face_center[1]

def get_y_angle(facecenter, screencenter):
    coordonnes = [-(facecenter[0] - screencenter[0]),-(facecenter[1] - screencenter[1])]
    return (coordonnes[1]*90)/screencenter[1]

def get_face_distance(face_center, screen_center):
    return sqrt((screen_center[0]-face_center[0])**2 + (screen_center[1]-face_center[1])**2)

def detect_bounding_box(vid):
    gray_image = cv2.cvtColor(vid, cv2.COLOR_BGR2GRAY)
    screen_center = (int((gray_image.shape[1])/2), int((gray_image.shape[0])/2))
    cv2.circle(vid, screen_center, 50, (255, 0, 0), 1)
    faces = face_classifier.detectMultiScale(gray_image, 1.1, 5, minSize=(40, 40))

    nearest_face = [[0,0], 1000000000000]
    for (x, y, w, h) in faces:
        face_center = (int(x+(w/2)), int(y+(h/2)))
        
        if nearest_face[1] >= get_face_distance(face_center, screen_center):
            nearest_face = [face_center , get_face_distance(face_center, screen_center)]
        cv2.circle(vid, face_center, 10, (255, 0, 0),10)
        if is_centered(face_center, screen_center):
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 255, 0), 4)
        else : 
            cv2.rectangle(vid, (x, y), (x + w, y + h), (0, 0, 255), 4)
        #print(get_y_angle(face_center, screen_center))
        
    return faces, nearest_face, screen_center 

fenetre = Tk()
label = Label(fenetre, text="visage")

################ Récupération des micros ################


################ Boucle main ################
while True:

    result, video_frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    faces, nearest_face, screen_center = detect_bounding_box(
        video_frame
    )  # apply the function we created to the video frame

    cv2.imshow(
        "My Face Detection Project", video_frame
    )  # display the processed frame in a window named "My Face Detection Project"

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    if nearest_face == [[0,0], 1000000000000]:
        label.config(text = "No Faces detected")
    elif is_centered(nearest_face[0], screen_center):
        label.config(text = "Nearest Face is centered")
    else:
        label.config(text = "Nearest Face Angle : " + str(get_y_angle(nearest_face[0], screen_center)/2))

    write_angle(ser, "P", "+", str(int(get_y_angle(nearest_face[0], screen_center)/2)))

    label.pack()

    fenetre.update()

fenetre.mainloop()
video_capture.release()
cv2.destroyAllWindows()
ser_motor.close()
ser_mic.close()