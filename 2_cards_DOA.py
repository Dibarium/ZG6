import serial

ser0 = serial.Serial(
    port='/dev/ttyACM0',  # Connection au port série du PC
    baudrate=115200,
    #timeout=10             # Timeout en secondes
)

ser1 = serial.Serial(
    port='/dev/ttyACM1',  # Connection au port série du PC
    baudrate=115200,
    #timeout=10             # Timeout en secondes
)

try:
    while True:
        if ser0.in_waiting > 0:      # Vérifie s'il y a des données disponibles à lire
            
            char = ser0.read_until(b'X')
            print(char.decode())

            if char:
                ser1.write(char)


except KeyboardInterrupt:
    print("Lecture interrompue par l'utilisateur")
finally:
    ser0.close()
    ser1.close()