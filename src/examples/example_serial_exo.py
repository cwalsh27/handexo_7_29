from nml_hand_exo import HandExo, SerialComm

# Serial usage
port = "COM6"
baudrate = 115200

comm = SerialComm(port=port, baudrate=baudrate)
exo = HandExo(comm, verbose=True)

try:
    exo.connect()
    print(exo.version())
except Exception as e:
    print(f"Error: {e}")
finally:
    exo.close()