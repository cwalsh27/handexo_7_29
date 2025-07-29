import time
import busio
import board
import usb_cdc

class USBSerialReader(object):
    """ Read a line from USB Serial (up to end_char), non-blocking, with optional echo 
    Depending on the microcontroller board used, the hardware may support multiple serial 
    communication methods. The RP2040 Pico has a "Console Direct Communication" (CDC)

    Parameters:
    -----------
    name              : (str)     The custom name for the class
    use_uart          : (bool)    The IP address to use to start the server
    port              : (int)     Port number on ip address to allow client connections
    rate              : (int)     Frequency of publishing rate for data to IP address
    simulate_hardware : (bool)    Allow physical control of hardware
    verbose           : (bool)    Enable/disable verbose output text to the terminal

    """
    def __init__(self, 
                 name="USBSerialReader",
                 use_UART=True, # If false, uses console
                 baudrate=9600,
                 terminator='\r', 
                 command_delimiter=";",  # multiple commands should be parsed too
                 argument_delimiter=":",
                 auto_connect=False,
                 verbose=False,
                ):
        self.name = name
        self.use_UART = use_UART
        self.baudrate = baudrate
        self.terminator =terminator.encode("utf-8")
        self.command_delimiter = command_delimiter
        self.argument_delimiter = argument_delimiter
        self.verbose = verbose

        self.serial = None
        self.connected = False
        self._in_data = bytearray()
        self.buffer = b""
        self._out_data = []
        
        if auto_connect:
            self.connect()
    
    def connect(self):
        if self.connected:
            if self.verbose:
                self.logger("Already connected.")
            return

        try:
            if self.use_UART:
                import board, busio
                self.serial = busio.UART(board.GP16, board.GP17, baudrate=self.baudrate, timeout=0)
            else:
                import usb_cdc
                self.serial = usb_cdc.data  # USB CDC data channel

            self.connected = True
            if self.verbose:
                self.logger("Serial connection established.")
        except Exception as e:
            self.serial = None
            self.connected = False
            self.logger(f"Connection failed: {e}")

    def disconnect(self):
        self.serial = None
        self.connected = False
        self.logger("Disconnected from serial.")
        
            
    @property
    def out_data(self):
        data = []
        
        # Decode the bytes currently stored in the buffer
        temp = self._out_data.decode("utf-8")
        
        # If a newline,return, or backspace character exists anywhere, those are not being parsed so we will remove them
        temp  = temp.replace('\n','').replace('\r','').replace('\b','')
        
        # Check if multiple commands were sent by splitting the message by the command delimiter
        cmds = temp.split(self.command_delimiter)
        
        for cmd in cmds:
            #data = cmd.split(self.argument_delimiter, 1)
            cmd_msg = cmd.split(self.argument_delimiter) # Want all possible arguments, not just the first

            if cmd_msg[-1] == '': del cmd_msg[-1] # Remove last element in arrray if empty            
            data.append(cmd_msg)
                    
        self._out_data = []
        return data
        
    def logger(self, *argv):
        msg = ''.join(argv)
        print("[{:.3f}][{}] {}".format(time.monotonic(), self.name, msg))

    def update(self):
        """ When this function runs, it checks whether any bytes are waiting in the USB CDC buffer, and fills in 
            either the in_data or out_data buffer if the terminal byte is read.
        """
        if not self.connected:
            return

        while self.serial.in_waiting > 0:
            try:
                chunk = self.serial.read(self.serial.in_waiting)
                self._in_data += chunk
                if self.verbose:
                    self.logger(f"Read {len(chunk)} bytes from serial: {chunk}")
                # Check if the terminator byte is in the received data
                if self.terminator in self._in_data:
                    if self.verbose:
                        self.logger(f"Terminator '{self.terminator.decode('utf-8')}' found in data.")
                    # Split the data at the terminator
                    parts = self._in_data.split(self.terminator)
                    # Keep all parts except the last one (which may be incomplete)
                    for part in parts[:-1]:
                        if part:  # Only process non-empty parts
                            self._out_data = part
                    # Keep the last part as it may contain incomplete data
                    self._in_data = parts[-1]

            except Exception as e:
                print(f"[USBSerialReader] Error reading serial: {e}")


    def send(self, msg):
        """ Send a message to the connected serial device
        """
        if not self.connected or not self.serial:
            self.logger("Send failed — serial not connected.")
            return

        if not msg.endswith("\n"):
            msg += "\n"
        try:
            self.serial.write(msg.encode("utf-8"))
            if self.verbose:
                self.logger(f"Sent: {msg.strip()}")
        except Exception as e:
            self.logger(f"Send error: {e}")
