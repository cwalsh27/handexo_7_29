import socket
import serial
import asyncio
import time


class BaseComm:
    def connect(self): pass
    def disconnect(self): pass
    def send(self, message: str): pass
    def receive(self) -> str: pass
    def is_connected(self) -> bool: pass


class TCPComm(BaseComm):
    def __init__(self, ip, port=5001, timeout=5, verbose=False):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.verbose = verbose

    def connect(self):
        try:
            if self.verbose:
                print(f"Attempting to connect to {self.ip}:{self.port} with timeout {self.timeout} seconds")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.ip, self.port))
            if self.verbose:
                print("Connection established")
        except socket.error as e:
            raise ConnectionError(f"Failed to connect to {self.ip}:{self.port} - {e}")


    def close(self):
        if self.sock:
            self.sock.close()

    def send(self, message: str):
        self.sock.sendall(message.encode())

    def receive(self) -> str:
        return self.sock.recv(1024).decode().strip()

    def is_connected(self) -> bool:
        return self.sock is not None


class SerialComm(BaseComm):
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.device = None
        self.command_delimiter = '\n'

    def connect(self):
   
        if self.verbose:
            print(f"Attempting to connect to {self.port}, baudrate {self.baudrate} with timeout {self.timeout} seconds")

        self.device = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        if self.device.is_open:
            if self.verbose:
                print("Connection successful")

    def close(self):
        if self.device and self.device.is_open:
            self.device.close()

    def send(self, message: str):
        self.device.write(message.encode())

    def receive(self) -> str:
        # Read bytes in the buffer until none are available
        # if not self.device or not self.device.is_open:
        #     raise ConnectionError("Serial device is not connected")
        # # Wait for a newline character to indicate end of message
        # self.device.flushInput()
        # self.device.flushOutput()
        # # Read until newline or timeout
        # # Note: This assumes the device sends a newline-terminated response

        # #return self.device.read_until(b'\n').decode().strip()
        # return self.device.readline().decode().strip()
        # #return self.sock.recv(1024).decode().strip()
        try:
            if self.device.in_waiting:
                response = self.device.read_until(self.command_delimiter.encode())
                if self.verbose:
                    print(f"Received: {response}")
                response = response.decode().replace(';','\n').strip()
                #print(f"stripped response: {response}")
                return response
        except Exception as e:
            print(f"[ERROR] Failed to read response: {e}")
        return ""


    def is_connected(self) -> bool:
        return self.device and self.device.is_open


class LowLevelTCPServer:
    def __init__(self, ip, port, serial_interface, logger, verbose=False):
        self.ip = ip
        self.port = port
        self.serial = serial_interface
        self.logger = logger
        self.verbose = verbose
        self.loop = asyncio.get_event_loop()

    async def handle_client(self, client_sock):
        addr = client_sock.getpeername()
        self.logger(f"[TCP] Client connected: {addr}")
        client_sock.setblocking(False)

        try:
            while True:
                data = await self.loop.sock_recv(client_sock, 1024)
                if not data:
                    break

                msg = data.decode("utf-8").strip()
                if self.verbose:
                    self.logger(f"[TCP] Received: {msg}")

                # Send command to serial
                self.serial.send(msg)

                # Wait for serial response
                response = await self.wait_for_serial_response(timeout=1.0)
                if response:
                    response_data = (response + "\n").encode("utf-8")
                    await self.loop.sock_sendall(client_sock, response_data)
                else:
                    await self.loop.sock_sendall(client_sock, b"[Timeout]\n")

        except Exception as e:
            self.logger(f"[TCP] Error: {e}", warning=True)
        finally:
            self.logger(f"[TCP] Client disconnected: {addr}")
            client_sock.close()

    async def wait_for_serial_response(self, timeout=1.0):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            self.serial.update()
            if self.serial._out_data:
                msg_list = self.serial.out_data
                # Flatten and join response
                return "\n".join(
                    str(item)
                    for sublist in msg_list if isinstance(sublist, list)
                    for item in sublist
                ).strip()
            await asyncio.sleep(0.01)
        return None

    async def run(self):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.bind((self.ip, self.port))
        server_sock.listen(5)
        server_sock.setblocking(False)
        self.logger(f"[TCP] Listening on {self.ip}:{self.port}")

        while True:
            client_sock, _ = await self.loop.sock_accept(server_sock)
            self.loop.create_task(self.handle_client(client_sock))