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
    #ser0.write(b'test')
    ser1.write(b'testtest')
    ser1.write(b'test')
    
    while True : 
        char = ser0.read(3)
        print("Received from ser0:", char)

        if char:
            #ser0.write(char)
            ser1.write(char)

except KeyboardInterrupt:
    print("Lecture interrompue par l'utilisateur")

finally:
    ser0.close()
    ser1.close()
