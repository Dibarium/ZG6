import serial

# Configuration du port série
ser = serial.Serial(
    port = '/dev/ttyACM0',        # Connection au port série du PC
    baudrate = 115200,            # Assurez-vous que le baudrate correspond à celui configuré sur votre STM32F411
    timeout = 1                   # Timeout en secondes
)

try:
    print("test")
    char = ""
    i = 0
    while True:
        if ser.in_waiting > 0:      # Vérifie s'il y a des données disponibles à lire
            print(i,": ser.in_waiting > 0")
            i+=1
            #buffer = ser.read(2)      # Lit un caractère du port série
            # if buffer == '\0': # Si on récupère les caractères nuls de fin de chaine

            #     print(char.decode())    # Affiche le caractère (décodé en utf-8)
            #     char = "" # Vide la liste de caractères
            # else:
            #     char = char + buffer # Incrémente la liste de caractères
            char = ser.read_until(b'X')
            print(char.decode())


except KeyboardInterrupt:
    print("Lecture interrompue par l'utilisateur")
finally:
    ser.close()                     # Ferme le port série
