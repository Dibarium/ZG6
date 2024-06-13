import tkinter as tk
from tkinter import ttk
import serial
import cv2
from math import sqrt
import time


#Serial
# try:
#     ser = serial.Serial(
#         port='/dev/ttyACM1',
#         baudrate=115200,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         bytesize=serial.EIGHTBITS
#     )
# except:
#     ser = serial.Serial(
#     port='/dev/ttyACM0',
#     baudrate=115200,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS
# )
# ser.close()

ser0 = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        #timeout=10             # Timeout en secondes

    )
ser1 = serial.Serial(
        port='/dev/ttyACM1',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        #timeout=10             # Timeout en secondes

    )

#DOA
def get_yaw_angle():
    if ser0.in_waiting > 0:      # Vérifie s'il y a des données disponibles à lire
        char = ser0.read_until(b'X')
        char = char[:-1]
        return char


#CV2 CONFIG
face_classifier = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)
video_capture = cv2.VideoCapture(2)

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

def convertforserial(pitch, yaw):
    result = ""
    #pitch part
    if int(pitch)>=0:
        result+= "+"
        result += ''.join(["0" for i in range(0, 3-len(str(pitch)))])
    else:
        pitch = str(int(pitch)*-1)
        result+= "-"
        result += ''.join(["0" for i in range(0, 3-len(str(pitch)))])
    result += pitch

    #yaw part
    if int(yaw)>=0:
        result += "+"
        result+= ''.join(["0" for i in range(0, 3-len(yaw))])
    else:
        yaw = str(int(yaw)*-1)
        result += "-"
        result+= ''.join(["0" for i in range(0, 3-len(yaw))])
    result += yaw
    return result

#TKinter
class Application(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Choix du Mode")
        self.geometry("300x200")
        
        self.create_widgets()
        
    def create_widgets(self):
        ttk.Label(self, text="Choisissez un mode:").grid(row=1, column=1, columnspan=2, pady=10)

        manual_button = ttk.Button(self, text="Mode Manuel", command=self.open_manual_mode)
        manual_button.grid(row=2, column=1, pady=10)

        auto_button = ttk.Button(self, text="Mode Automatique", command=self.open_auto_mode)
        auto_button.grid(row=2, column=2, pady=10)
    
    def open_manual_mode(self):
        self.withdraw()  # Masquer la fenêtre principale
        ManualModeWindow(self)
    
    def open_auto_mode(self):
        self.withdraw()  # Masquer la fenêtre principale
        AutoModeWindow(self)

class ManualModeWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.master = master  # Conserver une référence à la fenêtre principale
        self.title("Mode Manuel")
        self.geometry("600x400")
        
        ttk.Label(self, text="Pitch:").grid(row=0, column=0, pady=5, padx=5, sticky="e")
        self.pitch = ttk.Entry(self)
        self.pitch.grid(row=0, column=1, pady=5, padx=5, sticky="w")

        ttk.Label(self, text="Yaw:").grid(row=1, column=0, pady=5, padx=5, sticky="e")
        self.yaw = ttk.Entry(self)
        self.yaw.grid(row=1, column=1, pady=5, padx=5, sticky="w")

        valider_button = ttk.Button(self, text="Valider", command=self.valider)
        valider_button.grid(row=2, column=3, pady=10, padx=10, sticky="se")
    
        return_button = ttk.Button(self, text="Retour", command=self.on_return)
        return_button.grid(row=2, column=1, pady=10, padx=10, sticky="se")
    
    def on_return(self):
        self.destroy()
        self.master.deiconify()  # Afficher la fenêtre principale
    
    def valider(self):
        ser.open()
        pitch = self.pitch.get()
        yaw = self.yaw.get()
        serialised = convertforserial(pitch, yaw)
        print(serialised)
        ser.write(serialised.encode())
        ser.close()
        


    

class AutoModeWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.master = master  # Conserver une référence à la fenêtre principale
        self.title("Mode Automatique")
        self.geometry("600x400")
        
        pitchtext = tk.StringVar()
        pitchtext.set("Angle Pitch : ")
        
        yawtext = tk.StringVar()
        yawtext.set("Angle Yaw : ")

        ttk.Label(self, textvariable=pitchtext).grid(row=0, column=0, pady=10, padx=10)
        ttk.Label(self, textvariable=yawtext).grid(row=1, column=0, pady=10, padx=10)

        return_button = ttk.Button(self, text="Retour", command=self.on_return)
        return_button.grid(row=2, column=0, pady=10, padx=10, sticky="se")
        ser.open()

        iterateur = 0
        tableau = []
        while True:
            iterateur+= 1
            print(iterateur)
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
            
            if iterateur%50 == 0:
                mead = str(int(sum(tableau)/len(tableau)))
                serialised = convertforserial(mead, "0")
                print(serialised)
                ser.write(serialised.encode())
                
                tableau = []
                iterateur = 0
                
            else:
                tableau.append(int(get_y_angle(nearest_face[0], screen_center)/2))
            

            """
            if nearest_face == [[0,0], 1000000000000]:
                pitchtext.set("No Faces detected")
                yawtext.set("No Faces detected")
                serialised = convertforserial("0", "0")
                print(serialised)
                ser.write(serialised.encode())

            elif is_centered(nearest_face[0], screen_center):
                pitchtext.set("Nearest face is centered")
                yawtext.set("Nearest face is centered")
                serialised = convertforserial("0", "0")
                print(serialised)
                ser.write(serialised.encode())

            else:
                pitchtext.set("Pitch angle : " + str(get_y_angle(nearest_face[0], screen_center)/2))
                yawtext.set("Yaw angle : " + str(get_y_angle(nearest_face[0], screen_center)/2))
            
                pitch = str(int(get_y_angle(nearest_face[0], screen_center)/2))
                if pitch == "45":
                    pitch = "0"
                yaw = "0"
                serialised = convertforserial(pitch, yaw)
                print(serialised)
                ser.write(serialised.encode())
                """

            app.update()
        ser.close()

    def on_return(self):
        ser.close()
        cv2.destroyAllWindows()
        self.destroy()
        self.master.deiconify()  # Afficher la fenêtre principale

if __name__ == "__main__":
    app = Application()
    app.mainloop()

