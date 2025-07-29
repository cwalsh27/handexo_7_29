import ipaddress
import os
import asyncio
import time
import wifi  # Comment out to avoid issues on non-WiFi boards
import socketpool
from adafruit_httpserver import Server, Request, Response, POST

from webpage import WebPage
from rgbled import RGBLED
from usbserialreader import USBSerialReader

MAXBUF = 256


def url_decode(encoded_str):
    from binascii import unhexlify
    result = ""
    i = 0
    while i < len(encoded_str):
        if encoded_str[i] == '%':
            hex_val = encoded_str[i + 1:i + 3]
            try:
                result += chr(int(hex_val, 16))
                i += 3
            except ValueError:
                result += '%'
                i += 1
        elif encoded_str[i] == '+':
            result += ' '
            i += 1
        else:
            result += encoded_str[i]
            i += 1
    return result


class GUIState:
    def __init__(self, n_motors=6):
        self.dark_mode = False
        self.connected = False
        self.port = ""
        self.baud = 9600
        self.operating_mode = "GESTURE_FIXED"
        self.motor_values = [0] * n_motors
        self.log_text = ""


class AsyncController(object):
    """
    This object handles serial command inputs and directs the servo positions
    according to the commands given, optionally via TCP/IP.
    """

    def __init__(self, name='ExoController',
                 ip=None,
                 http_port=5000,
                 command_port=5001,
                 rate=50,
                 use_wifi=True,
                 subnet_mask="255.255.255.0",
                 gateway="192.168.1.1",
                 dns="8.8.8.8",
                 use_serial=True,
                 baudrate=57600,
                 use_uart=True,
                 command_delimiter=";",
                 argument_delimiter="~",
                 verbose=False):

        self.name = name
        self.ip = ip
        self.http_port = http_port
        self.command_port = command_port
        self.rate = rate
        self.use_wifi = use_wifi
        self.subnet_mask = subnet_mask
        self.gateway = gateway
        self.dns = dns
        self.use_serial = use_serial
        self.use_uart = use_uart
        self.command_delimiter = command_delimiter
        self.argument_delimiter = argument_delimiter
        self.verbose = verbose

        self.tcp_clients = []
        self.server = None
        self.all_stop = False
        self.connected = False
        self.rgb = RGBLED(set_color=[30, 0, 0])
        self.gui_state = GUIState(n_motors=6)

        # Establish peripheral connection types
        self.logger("Setting up Serial support ...")
        if self.use_serial:
            self.serial = USBSerialReader(
                use_UART=self.use_uart,
                baudrate=baudrate,
                terminator='\r\n',
                command_delimiter=self.command_delimiter,
                argument_delimiter=self.argument_delimiter,
                auto_connect=True,
                # verbose=self.verbose)
                verbose=False)

        self.logger(f"{self.name} created.")

        # Sending first message after object intialization
        # self.serial.send("version\n")

    def logger(self, *argv, warning=False, level="debug", verbose=False):
        msg = ''.join(argv)
        prefix = "(Warning) " if warning else ""
        print("[{:.3f}][{}] {}{}".format(time.monotonic(), self.name, prefix, msg))

    async def main(self):
        # if self.use_serial:
        # self.logger("USB Serial Parser set up. Reading serial commands")
        # asyncio.create_task(self.serial_client(50))

        if self.use_wifi:
            wifi.radio.hostname = "handexo"
            if self.ip is not None:
                wifi.radio.set_ipv4_address(
                    ipv4=ipaddress.IPv4Address(self.ip),
                    netmask=ipaddress.IPv4Address(self.subnet_mask),
                    gateway=ipaddress.IPv4Address(self.gateway),
                    ipv4_dns=ipaddress.IPv4Address(self.dns)
                )
            if await self.connect_to_wifi():
                self.pool = socketpool.SocketPool(wifi.radio)
                self.start_http_server()
                # self.logger("Try: http://handexo.local (if mDNS supported)")

                # TCP Server
                self.logger(f"Starting TCP server on {self.ip}:{self.command_port}")
                asyncio.create_task(self.start_tcp_server())

        # asyncio.create_task(self.update(self.rate))

        if self.verbose:
            self.logger(f"{self.name} running!")

        while not self.all_stop:
            if self.use_wifi and self.server:
                try:
                    self.server.poll()
                except Exception as e:
                    self.logger(f"HTTP server error: {e}", warning=True)
            await asyncio.sleep(0)

    async def connect_to_wifi(self, max_retries=5):
        """ Makes an attempt to connect to the network given the credentials.

        Note: Since we are using the asyncronous server initialization, we don't need to refer to an object. We can just pass a boolean when
        the connection is successful or not
        """
        # ssid = os.getenv('SHULGACH_WIFI_SSID')
        # password = os.getenv('SHULGACH_WIFI_PASSWORD')
        ssid = os.getenv('CIRCUITPYTHON_WIFI_SSID')
        password = os.getenv('CIRCUITPYTHON_WIFI_PASSWORD')
        # ssid = os.getenv('NML_SSID')
        # password = os.getenv('NML_PASSWORD')
        # ssid = os.getenv('CMU_SSID')
        # password = os.getenv('CMU_PASSWORD')

        if not ssid or not password:
            self.logger("Missing Wi-Fi credentials in settings.toml", warning=True)
            return False

        self.logger(f"Attempting to connect to {ssid} Wi-Fi network...")

        for attempt in range(1, max_retries + 1):
            try:
                wifi.radio.connect(ssid, password)
                while not wifi.radio.ipv4_address:
                    self.logger("Waiting for IP address...")
                    await asyncio.sleep(1)

                self.ip = str(wifi.radio.ipv4_address)
                self.logger(f"Connected to {ssid}! IP: {self.ip}")
                self.rgb.set_color([0, 100, 0])
                return True
            except Exception as e:
                self.logger(f"Attempt {attempt} failed: {e}", warning=True)
                await asyncio.sleep(2)

        self.logger("Failed to connect to Wi-Fi after multiple attempts", warning=True)
        return False

    async def update(self, interval=200):
        while not self.all_stop:
            if self.queue:
                msg = self.queue.pop(0)
                if self.verbose:
                    self.logger(f"Reading {msg} from list")
                await self.parse_command([msg])
            await asyncio.sleep(1 / interval)

    async def serial_client(self, rate):
        """ Read serial messages from the client and places them in queue """
        while self.all_stop != True:
            self.serial.update()

            if self.serial._out_data:
                # Skip the queueing structure and handle commands as soon as they come
                msg_list = self.serial.out_data

                for msg in msg_list:
                    if self.verbose:
                        self.logger(f"Serial message: {msg}")

                    # # Send serial message to the connected TCP clients
                    # if self.use_wifi and self.tcp_clients:
                    #     self.logger(f"Sending to {len(self.tcp_clients)} TCP clients: {msg}")
                    #     for writer in list(self.tcp_clients):
                    #         try:
                    #             writer.write((msg + "\n").encode("utf-8"))
                    #             await writer.drain()
                    #         except Exception as e:
                    #             self.logger(f"Error sending to TCP client: {e}", warning=True)
                    #             self.tcp_clients.remove(writer)  # Remove disconnected client
                    # else:
                    #     if self.verbose:
                    #         self.logger("No TCP clients connected, skipping send", warning=True)

                    # await self.parse_command(data)
            await asyncio.sleep(1 / rate)

    async def handle_client(self, sock):
        sock.setblocking(False)
        buffer = bytearray(MAXBUF)
        message_buffer = b""

        while not self.all_stop:
            try:
                nbytes = sock.recv_into(buffer, MAXBUF)
                if nbytes == 0:
                    break  # connection closed

                message_buffer += buffer[:nbytes]

                while b"\n" in message_buffer:
                    line, message_buffer = message_buffer.split(b"\n", 1)
                    command = line.decode().strip()
                    if self.verbose:
                        self.logger(f"TCP command: {command}")

                    self.serial.send(command)

                    # Wait for response
                    timeout = 1.0
                    start = time.monotonic()
                    response = ""
                    while time.monotonic() - start < timeout:
                        self.serial.update()
                        if self.serial._out_data:
                            msg_list = self.serial.out_data
                            response = "\n".join(
                                str(item) for sublist in msg_list if isinstance(sublist, list) for item in sublist
                            )
                            break
                        await asyncio.sleep(0.01)

                    if response:
                        if self.verbose:
                            self.logger(f"Sending response: {response}")
                        try:
                            sock.send(response.encode("utf-8"))
                        except OSError as e:
                            self.logger(f"Client socket error: {e}", warning=True)
                            break

            except OSError as e:
                if e.errno == 11:  # EAGAIN
                    await asyncio.sleep(0.01)
                    continue
                else:
                    self.logger(f"Client socket error: {e}", warning=True)
                    break
            except Exception as e:
                self.logger(f"Client error: {e}", warning=True)
                break

        sock.close()
        if sock in self.tcp_clients:
            self.tcp_clients.remove(sock)
        self.logger("Client disconnected")

    async def parse_command(self, cmd):
        # Implement command handling logic here
        if self.verbose:
            self.logger(f"Parsing: {cmd}")

    def start_http_server(self):
        self.server = Server(self.pool, "/static", debug=True)
        self.setup_routes()
        self.server.start(self.ip)
        self.logger(f"HTTP server at http://{self.ip}")

    async def start_tcp_server(self):
        server_sock = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
        server_sock.bind((self.ip, self.command_port))
        server_sock.listen(5)
        server_sock.setblocking(False)

        self.logger(f"TCP server listening on {self.ip}:{self.command_port}")

        while not self.all_stop:
            try:
                client_sock, addr = server_sock.accept()
                self.logger(f"TCP client connected: {addr}")
                self.tcp_clients.append(client_sock)
                asyncio.create_task(self.handle_client(client_sock))
            except Exception:
                await asyncio.sleep(0.1)

    def setup_routes(self):
        """
        Setup https handling routes
        """

        @self.server.route("/")
        def base(request: Request):
            return Response(request, WebPage(self.gui_state), content_type='text/html')

        @self.server.route("/", POST)
        def handle_post(request: Request):
            try:
                form_data = request.form_data
                log_lines = []

                # --- Handle dark mode toggle ---
                self.gui_state.dark_mode = form_data.get("dark_mode") == "on"

                # --- Handle custom serial command ---
                direct_msg = url_decode(form_data.get("direct_msg", "").strip())
                if direct_msg:
                    log_lines.append(f"[Custom] {direct_msg}")
                    if self.use_serial:
                        self.serial.send(direct_msg + "\n")
                    self.gui_state.log_text += "\n" + "\n".join(log_lines)
                    return Response(request, WebPage(self.gui_state), content_type='text/html')

                # --- Handle Connect/Disconnect toggle ---
                if "connect" in form_data:
                    if not self.gui_state.connected:
                        self.serial.connect()
                        self.gui_state.connected = True
                    else:
                        self.serial.disconnect()
                        self.gui_state.connected = False
                    log_lines.append("Connected" if self.gui_state.connected else "Disconnected")

                # --- Update operating mode if changed ---
                selected_mode = form_data.get("mode", self.gui_state.operating_mode)
                if selected_mode != self.gui_state.operating_mode:
                    self.gui_state.operating_mode = selected_mode
                    log_lines.append(f"Mode: {selected_mode}")

                # --- Handle motor updates ---
                for i in range(len(self.gui_state.motor_values)):
                    angle_val = form_data.get(f"motor{i}") or form_data.get(f"angle{i}")
                    if angle_val is not None:
                        try:
                            val = int(angle_val)
                            if self.gui_state.motor_values[i] != val:
                                self.gui_state.motor_values[i] = val
                                log_lines.append(f"Motor {i}: {val}")
                                if self.use_serial:
                                    self.serial.send(f"gc:motor{i}:{val}\n")
                                break  # Only one motor change at a time
                        except ValueError:
                            log_lines.append(f"[Invalid] motor{i} value: {angle_val}")

                # --- Update log text if anything changed ---
                if log_lines:
                    self.gui_state.log_text += "\n" + "\n".join(log_lines)

                return Response(request, WebPage(self.gui_state), content_type='text/html')

            except Exception as e:
                self.gui_state.log_text += f"\nError: {e}"
                return Response(request, WebPage(self.gui_state), content_type='text/html')

    def start(self):
        """ Makes a call to the asyncronous library to run a main routine """
        asyncio.run(self.main())  # Need to pass the async function into the run method to start

    def stop(self):
        """ Sets a flag to stop running all tasks """
        self.all_stop = True


if __name__ == "__main__":
    # Set the sevrer IP
    ip = os.getenv('CIRCUITPYTHON_WIFI_IP')
    http_port = os.getenv('CIRCUITPYTHON_HTTP_PORT')
    tcp_port = os.getenv('CIRCUITPYTHON_TCP_PORT')

    # Create controller
    controller = AsyncController(ip=ip, command_port=tcp_port, use_wifi=True, use_serial=True, verbose=True)

    # Begin server
    controller.start()

