import serial

# Configuration du port série
ser = serial.Serial(
    port = '/dev/ttyACM0',        # Connection au port série du PC
    baudrate = 115200,            # Assurez-vous que le baudrate correspond à celui configuré sur votre STM32F411
    timeout = 1                   # Timeout en secondes
)

try:
    print("test")
    while True:
        if ser.in_waiting > 0:      # Vérifie s'il y a des données disponibles à lire
            print("ser.in_waiting > 0")
            char = ser.read(3)      # Lit un caractère du port série
            print(char.decode())    # Affiche le caractère (décodé en utf-8)
except KeyboardInterrupt:
    print("Lecture interrompue par l'utilisateur")
finally:
    ser.close()                     # Ferme le port série
