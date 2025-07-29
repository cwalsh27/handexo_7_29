import numpy as np
import time
import re

from nml_hand_exo.interfaces import BaseComm, LowLevelTCPServer
#from _filters import OrientationFilter

class OrientationFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.last_time = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def update(self, accel, gyro, current_time=None):
        """
        Updates orientation using accelerometer and gyroscope data.

        Args:
            accel: List or tuple of 3 floats (ax, ay, az) in m/s^2.
            gyro: List or tuple of 3 floats (gx, gy, gz) in rad/s.
            current_time: (Optional) float timestamp. If None, uses time.monotonic().
        Returns:
            dict with keys: 'roll', 'pitch', 'yaw' (all in degrees).
        """
        ax, ay, az = accel
        gx, gy, gz = gyro
        now = current_time or time.monotonic()

        if self.last_time is None:
            self.last_time = now
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        dt = now - self.last_time
        self.last_time = now

        # Accelerometer-based roll/pitch estimate
        accel_roll = np.arctan2(ay, az)
        accel_pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # Gyro integration
        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        # Complementary filter fusion
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch

        return {
            'roll': float(np.degrees(self.roll)),
            'pitch': float(np.degrees(self.pitch)),
            'yaw': float(np.degrees(self.yaw))
        }
    
class HandExo(object):
    """
    Class to control the NML Hand Exoskeleton via serial communication.

    Features:
      - Enable/disable motors
      - Move motors to specific angles
      - Query status (angle, torque, current)
      - Configure velocity and acceleration
      - Retrieve device information
      - Send low-level serial commands

    """

    def __init__(self, comm: BaseComm, name='NMLHandExo', command_delimiter: str = '\n', send_delay: float = 0.01,
                 auto_connect=False, verbose: bool = False):
        """ 
        Initializes the HandExo interface.
        
        Args:
            name (str): Name of the exoskeleton instance.
            command_delimiter (str): Delimiter used to separate commands (default is '\n').
            send_delay (float): Delay in seconds after sending a command to allow processing (default is 0.01).
            verbose (bool): If True, enables verbose logging of commands and responses (default is False).

        """
        self.name = name
        self.device = comm
        self.command_delimiter = command_delimiter
        self.send_delay = send_delay
        self.verbose = verbose
        self.device.verbose = verbose
        self.imu_filter = OrientationFilter()

        if auto_connect:
            self.device.connect()

    def logger(self, *argv, warning: bool = False):
        """ 
        Robust debugging print function
        
        Args:
            *argv             : (str) Messages to log.
            warning           : (bool) If True, prints the message in yellow.

        """
        if self.verbose:
            msg = ''.join(argv)
            msg = f"[{time.monotonic():.3f}][{self.name}] {msg}"

            # If a warning, print the text in yellow
            msg = f"\033[93m{msg}\033[0m" if warning else msg
            print(msg)

    def connect(self):
        """
        """
        self.device.connect()

    def send_command(self, cmd: str):
        """
        Sends a command to the exoskeleton over the serial connection.

        Args:
            cmd (str): Command to send to the exoskeleton.

        """
        if not cmd.endswith(self.command_delimiter):
            cmd += self.command_delimiter
        try:
            self.device.send(cmd)
            self.logger(f"Sent: {cmd.strip()}")
            time.sleep(self.send_delay)  # Allow time for the command to be processed
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

    def _receive(self):
        """
        Reads a response from the exoskeleton over the serial connection.
        
        Returns:
            str: The response from the exoskeleton, or an empty string if no response.

        """
        resp = self.device.receive()
        if resp:
            resp = resp.strip()
            self.logger(f"Received: {resp}")
            return resp

        # try:
        #     if self.device.in_waiting:
        #         response = self.device.receive(self.command_delimiter.encode()).decode().strip()
        #         self.logger(f"Received: {response}")
        #         return response
        # except Exception as e:
        #     print(f"[ERROR] Failed to read response: {e}")
        # return ""

    def enable_motor(self, motor_id: (int or str) = 'all'):
        """
        Enables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable.

        Returns:
            None

        """
        self.send_command(f"enable:{motor_id}")

    def is_enabled(self, motor_id: (int or str) = 'all') -> bool:
        """
        Checks if the specified motor is enabled.

        Args:
            motor_id (int or str): ID of the motor to check.

        Returns:
            bool: True if the motor is enabled, False otherwise.

        """
        self.send_command(f"get_enable:{motor_id}")
        response = self._receive()
        return response.lower() == 'true' if response else False

    def disable_motor(self, motor_id: (int or str) = 'all'):
        """
        Disables the torque output for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable.

        Returns:
            None

        """
        self.send_command(f"disable:{motor_id}")

    def enable_led(self, motor_id: (int or str) = 'all'):
        """
        Enables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to enable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:on")

    def disable_led(self, motor_id: (int or str) = 'all'):
        """
        Disables the LED for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to disable the LED for.

        Returns:
            None

        """
        self.send_command(f"led:{motor_id}:off")

    def help(self) -> str:
        """
        Sends a help command to the exoskeleton to retrieve available commands.

        Returns:
            str: A string containing the help information from the exoskeleton.

        """
        self.send_command("help")
        return self._receive()

    def version(self) -> str:
        """
        Gets the version of the exo
        """
        self.send_command("version")
        response = self._receive()

        if response:
            return response.strip().split(':')[1]
        return ""

    def home(self, motor_id: (int or str) = 'all'):
        """
        Sends a home command to all motors, unless a specific motor ID is provided.

        Args:
            motor_id (int or str): ID of the motor to home, or 'all' to home all motors."

        Returns:
            None

        """
        self.send_command(f"home:{motor_id}")

    def info(self) -> dict:
        """
        Parses the exoskeleton info response into a structured dictionary.
        """
        self.send_command("info")
        raw = self._receive()
        print(f"Raw return: {raw}")

        info = {}
        if not raw:
            return info

        try:
            lines = [line.strip() for line in raw.splitlines() if line.strip()]

            # First line should have name, version, and motor count
            header = lines[0]
            name_match = re.search(r"Name:\s*([^\s]+)", header)
            version_match = re.search(r"Version:\s*([^\s]+)", header)
            motor_count_match = re.search(r"Number of Motors:\s*(\d+)", header)

            if name_match:
                info['name'] = name_match.group(1)
            if version_match:
                info['version'] = version_match.group(1)
            if motor_count_match:
                info['n_motors'] = int(motor_count_match.group(1))

            # Parse each motor line
            for line in lines[1:]:
                motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                if not motor_match:
                    continue

                motor_id = int(motor_match.group(1))
                motor_data = motor_match.group(2)

                # Parse the individual fields inside the { ... }
                motor_info = {}
                for part in motor_data.split(','):
                    key_val = part.strip().split(":", 1)
                    if len(key_val) != 2:
                        continue
                    key, val = key_val[0].strip(), key_val[1].strip()
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key == "angle":
                        motor_info["angle"] = float(val)
                    elif key == "limits":
                        limits_match = re.findall(r"[-+]?[0-9]*\.?[0-9]+", val)
                        motor_info["limits"] = [float(l) for l in limits_match]
                    elif key == "torque":
                        motor_info["torque"] = float(val)
                    elif key == "enabled":
                        motor_info["enabled"] = val.lower() == "true"
                    else:
                        motor_info[key] = val

                info[f"motor_{motor_id}"] = motor_info

            return info

        except Exception as e:
            print(f"[ERROR] Failed to parse info: {e}")
            return {}

    def get_baudrate(self, motor_id: (int or str) = 'all') -> int:
        """
        Retrieves the current baud rate of the serial connection.

        Returns:
            int: The current baud rate.

        """
        self.send_command(f"get_baud:{motor_id}")
        response = self._receive()
        return response

    def get_motor_velocity(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current velocity of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current velocity of the motor in degrees per second.

        """
        self.send_command(f"get_vel:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_motor_velocity(self, motor_id: (int or str), velocity: float):
        """
        Sets the velocity for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the velocity for.
            velocity (float): Desired velocity in degrees per second.

        Returns:
            None

        """
        self.send_command(f"set_vel:{motor_id}:{velocity}")

    def get_motor_acceleration(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current acceleration of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current acceleration of the motor in degrees per second squared.

        """
        self.send_command(f"get_accel:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_motor_acceleration(self, motor_id: (int or str), acceleration: float):
        """
        Sets the acceleration for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the acceleration for.
            acceleration (float): Desired acceleration in degrees per second squared.

        Returns:
            None

        """
        self.send_command(f"set_accel:{motor_id}:{acceleration}")

    def get_motor_angle(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the current relative angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current angle of the motor in degrees.

        """
        # print('were in')
        while True:
            self.send_command(f"get_angle:{motor_id}")
            raw = self._receive()
            # print('anybody home')
            while (raw is None) or ("relative" not in raw):
                # print("not on my watch")
                self.send_command("get_imu")
                raw = self._receive()
            print(f"Raw motor return: {raw}")
            

        # If the response is a digit, return it as a float
        #and raw.isdigit() or raw.isdecimal():
            if 'rad' not in raw:
                return float(raw.split(':')[-1].strip())
            


        info = {}
        if not raw:
            return info

        try:
            lines = [line.strip() for line in raw.splitlines() if line.strip()]

            # First line should have name, version, and motor count
            header = lines[0]

            # Parse each motor line
            for line in lines[1:]:
                motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                if not motor_match:
                    continue

                motor_id = int(motor_match.group(1))
                motor_data = motor_match.group(2)

                # Parse the individual fields inside the { ... }
                motor_info = {}
                for part in motor_data.split(','):
                    key_val = part.strip().split(":", 1)
                    if len(key_val) != 2:
                        continue
                    key, val = key_val[0].strip(), key_val[1].strip()
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key == "angle":
                        motor_info["angle"] = float(val)
                    else:
                        motor_info[key] = val

                info[f"motor_{motor_id}"] = motor_info

            return info

        except Exception as e:
            print(f"[ERROR] Failed to parse info: {e}")
            return {}

    def set_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the angle for.
            angle (float): Desired angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_angle:{motor_id}:{angle}"
        else:
            cmd = f"set_angle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_absolute_motor_angle(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the absolute angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Absolute angle of the motor in degrees.

        """
        self.send_command(f"get_absangle:{motor_id}")
        raw = self._receive()
        # If the response is a digit, return it as a float
        if raw and raw.isdigit() or raw.isdecimal():
            return float(raw)

        info = {}
        if not raw:
            return info

        try:
            lines = [line.strip() for line in raw.splitlines() if line.strip()]

            # First line should have name, version, and motor count
            header = lines[0]

            # Parse each motor line
            for line in lines[1:]:
                motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                if not motor_match:
                    continue

                motor_id = int(motor_match.group(1))
                motor_data = motor_match.group(2)

                # Parse the individual fields inside the { ... }
                motor_info = {}
                for part in motor_data.split(','):
                    key_val = part.strip().split(":", 1)
                    if len(key_val) != 2:
                        continue
                    key, val = key_val[0].strip(), key_val[1].strip()
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key == "angle":
                        motor_info["absolute_angle"] = float(val)
                    else:
                        motor_info[key] = val

                info[f"motor_{motor_id}"] = motor_info

            return info

        except Exception as e:
            print(f"[ERROR] Failed to parse info: {e}")
            return {}

    def set_absolute_motor_angle(self, motor_id: (int or str), angle: float):
        """
        Sets the absolute angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the absolute angle for.
            angle (float): Desired absolute angle in degrees.

        Returns:
            None

        """
        if isinstance(motor_id, str):
            cmd = f"set_absangle:{motor_id}:{angle}"
        else:
            cmd = f"set_absangle:{int(motor_id)}:{angle}"
        self.send_command(cmd)

    def get_home(self, motor_id: (int or str) = 'all') -> float:
        """
        Retrieves the home angle of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Home angle of the motor in degrees.

        """
        self.send_command(f"get_home:{motor_id}")
        raw = self._receive()
        print(f"Raw return: {raw}")

        # If the response is a digit, return it as a float
        if raw and raw.isdigit():
            return float(raw)

        info = {}
        if not raw:
            return info

        try:
            lines = [line.strip() for line in raw.splitlines() if line.strip()]

            # First line should have name, version, and motor count
            header = lines[0]
            name_match = re.search(r"Name:\s*([^\s]+)", header)
            version_match = re.search(r"Version:\s*([^\s]+)", header)
            motor_count_match = re.search(r"Number of Motors:\s*(\d+)", header)

            if name_match:
                info['name'] = name_match.group(1)
            if version_match:
                info['version'] = version_match.group(1)
            if motor_count_match:
                info['n_motors'] = int(motor_count_match.group(1))

            # Parse each motor line
            for line in lines[1:]:
                motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                if not motor_match:
                    continue

                motor_id = int(motor_match.group(1))
                motor_data = motor_match.group(2)

                # Parse the individual fields inside the { ... }
                motor_info = {}
                for part in motor_data.split(','):
                    key_val = part.strip().split(":", 1)
                    if len(key_val) != 2:
                        continue
                    key, val = key_val[0].strip(), key_val[1].strip()
                    if key == "id":
                        motor_info["id"] = int(val)
                    elif key == "home":
                        motor_info["angle"] = float(val)
                    else:
                        motor_info[key] = val

                info[f"motor_{motor_id}"] = motor_info

            return info

        except Exception as e:
            print(f"[ERROR] Failed to parse info: {e}")
            return {}

    def set_home(self, motor_id: (int or str), home_angle: float):
        """
        Sets the home angle for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the home angle for.
            home_angle (float): Desired home angle in degrees.

        Returns:
            None

        """
        self.send_command(f"set_home:{motor_id}:{home_angle}")

    def get_motor_torque(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current torque of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current torque of the motor in Newton-meters.

        """
        self.send_command(f"get_torque:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def get_motor_current(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current draw of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current draw of the motor in Amperes.

        """
        self.send_command(f"get_current:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def get_motor_current_limit(self, motor_id: (int or str)) -> float:
        """
        Retrieves the current limit of the specified motor.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            float: Current limit of the motor in Amperes.

        """
        self.send_command(f"get_current_lim:{motor_id}")
        response = self._receive()
        try:
            return float(response.split(':')[-1])
        except (ValueError, IndexError):
            print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
            return 0.0

    def set_current_limit(self, motor_id: (int or str), current_limit: float):
        """
        Sets the current limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the current limit for.
            current_limit (float): Desired current limit in Amperes.

        Returns:
            None

        """
        self.send_command(f"set_current_lim:{motor_id}:{current_limit}")

    def get_motor_status(self, motor_id: (int or str)) -> dict:
        """
        Retrieves the status of the specified motor, including angle, torque, current, velocity, and acceleration.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            dict: A dictionary containing the motor's status.

        """
        self.send_command(f"get_status:{motor_id}")
        response = self._receive()
        status = {}
        if response:
            try:
                parts = response.split(',')
                status['angle'] = float(parts[0].split(':')[-1])
                status['torque'] = float(parts[1].split(':')[-1])
                status['current'] = float(parts[2].split(':')[-1])
                status['velocity'] = float(parts[3].split(':')[-1])
                status['acceleration'] = float(parts[4].split(':')[-1])
                return status
            except (ValueError, IndexError):
                print(f"[ERROR] Invalid response for motor {motor_id}: {response}")
        return {}

    def get_motor_limits(self, motor_id: (int or str) = 'all') -> tuple:
        """
        Retrieves the limits for the specified motor, including minimum and maximum angles.

        Args:
            motor_id (int or str): ID of the motor to query.

        Returns:
            tuple: A tuple containing the minimum and maximum angles of the motor.

        """
        self.send_command(f"get_motor_limits:{motor_id}")
        raw = self._receive()  # Should get "Motor {id} limits:[{val},{val}]"

        if motor_id == 'all':
            info = {}
            if not raw:
                return info

            try:
                lines = [line.strip() for line in raw.splitlines() if line.strip()]

                # First line should have name, version, and motor count
                header = lines[0]

                # Parse each motor line
                for line in lines[1:]:
                    motor_match = re.match(r"Motor\s+(\d+):\s*\{(.+?)\}", line)
                    if not motor_match:
                        continue

                    motor_id = int(motor_match.group(1))
                    motor_data = motor_match.group(2)

                    # Parse the individual fields inside the { ... }
                    motor_info = {}
                    for part in motor_data.split(','):
                        key_val = part.strip().split(":", 1)
                        if len(key_val) != 2:
                            continue
                        key, val = key_val[0].strip(), key_val[1].strip()
                        if key == "id":
                            motor_info["id"] = int(val)
                        elif key == "Limits":
                            print(f"limits: {val}")
                            motor_info["lower_limit"] = float(val.split(',')[0].strip('[]'))
                            motor_info["upper_limit"] = float(val.split(',')[1].strip('[]'))
                        else:
                            motor_info[key] = val

                    info[f"motor_{motor_id}"] = motor_info

                return info

            except Exception as e:
                print(f"[ERROR] Failed to parse info: {e}")
                return {}
        else:
            limits = None
            if raw:
                try:
                    parts = raw.split(':')
                    if len(parts) == 3 and parts[0].startswith("Motor") and parts[1].strip() == "limits":
                        limits = tuple(map(float, parts[2].strip('[]').split(',')))
                    else:
                        print(f"[ERROR] Invalid response for motor {motor_id}: {raw}")
                except (ValueError, IndexError):
                    print(f"[ERROR] Failed to parse limits for motor {motor_id}: {raw}")
            return limits if limits else (None, None)

    def set_motor_upper_limit(self, motor_id: (int or str), upper_limit: float):
        """
        Sets the upper limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the upper limit for.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_upper_limit:{motor_id}:{upper_limit}")

    def set_motor_lower_limit(self, motor_id: (int or str), lower_limit: float):
        """
        Sets the lower limit for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the lower limit for.
            lower_limit (float): Desired lower limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_lower_limit:{motor_id}:{lower_limit}")

    def set_motor_limits(self, motor_id: (int or str), lower_limit: float, upper_limit: float):
        """
        Sets both the lower and upper limits for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the limits for.
            lower_limit (float): Desired lower limit in degrees.
            upper_limit (float): Desired upper limit in degrees.

        Returns:
            None

        """
        self.send_command(f"set_motor_limits:{motor_id}:{lower_limit}:{upper_limit}")

    def reboot_motor(self, motor_id: (int or str)):
        """
        Reboots the specified motor.

        Args:
            motor_id (int or str): ID of the motor to reboot.

        Returns:
            None

        """
        if motor_id == 'all':
            self.send_command("reboot:all")
        else:
            self.send_command(f"reboot:{motor_id}")

    def get_motor_mode(self) -> str:
        """
        Retrieves the current control mode of the specified motor.

        Returns:
            str: Current mode of the motor (e.g., "position", "velocity", "current_position").

        """
        self.send_command(f"get_motor_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_motor_mode(self, motor_id: (int or str), mode: str):
        """
        Sets the control mode for the specified motor.

        Args:
            motor_id (int or str): ID of the motor to set the mode for.
            mode (str): Desired control mode (e.g., "position", "velocity", "current_position").

        Returns:
            None

        """
        self.send_command(f"set_motor_mode:{motor_id}:{mode}")

    def get_exo_mode(self) -> str:
        """
        Retrieves the current operating mode of the exoskeleton.

        Returns:
            str: Current mode of the exoskeleton (e.g., "manual", "autonomous").

        """
        self.send_command("get_exo_mode")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_exo_mode(self, mode: str):
        """
        Sets the operating mode for the exoskeleton.

        Args:
            mode (str): Desired operating mode (e.g., "manual", "autonomous").

        Returns:
            None

        """
        self.send_command(f"set_exo_mode:{mode}")

    def get_gesture(self) -> str:
        """
        Retrieves the current gesture recognized by the exoskeleton.

        Returns:
            str: Current gesture (e.g., "open", "close", "pinch").

        """
        self.send_command("get_gesture")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""

    def set_gesture(self, gesture: str, state: str = "default"):
        """
        Sets the gesture for the exoskeleton.

        Args:
            gesture (str): Desired gesture (e.g., "open", "close", "pinch").

        Returns:
            None

        """
        self.send_command(f"set_gesture:{gesture}:{state}")

    def set_gesture_state(self, state: str):
        """
        Sets the state of the current gesture for the exoskeleton.

        Args:
            state (str): Desired state of the gesture (e.g., "default", "active").

        Returns:
            None

        """
        self.send_command(f"set_gesture_state:{state}")

    def get_gesture_list(self) -> list:
        """
        Retrieves the list of available gestures for the exoskeleton.

        Returns:
            list: A list of available gestures.

        """
        self.send_command("gesture_list")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip().split(',')
            except IndexError:
                print(f"[ERROR] Invalid response")
        return []

    def cycle_gesture(self):
        """
        Cycles through the available gestures for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture")

    def cycle_gesture_state(self):
        """
        Cycles through the states of the current gesture for the exoskeleton.

        Returns:
            None

        """
        self.send_command("cycle_gesture_state")

    def close(self):
        """
        Closes the serial connection to the exoskeleton.

        Returns:
            None

        """
        if self.device and self.device.is_connected():
            self.device.close()
            self.logger("Device connection closed.")

    def get_imu_data(self) -> dict:
        """
        Retrieves the IMU data from the exoskeleton.

        Receives a serial message with contents, as an example:
            "Temp: 20.28 C
             Accel: [-0.46, -0.42, 9.85]
             Gyro: [-0.00, 0.01, -0.00]"

        Returns:
            dict: A dictionary containing IMU data (e.g., accelerometer, gyroscope, magnetometer).

        """
        self.send_command("get_imu")
        response = self._receive()
        # print(response)
        while (response is None) or ("Heading" not in response):
            # print("not on my watch")
            self.send_command("get_imu")
            response = self._receive()
            # print(response)

        imu_data = {}
        if response:
            try:
                lines = [line.strip() for line in response.splitlines() if line.strip()]
                for part in lines:
                    if part.startswith("Temp:"):
                        imu_data['temperature'] = float(part.split(':')[-1].strip().replace('C', ''))
                    elif part.startswith("Accel:"):
                        accel_str = part.split(':')[-1].strip().replace(']', '').replace('[', '')
                        accel_str = accel_str.replace('m/s^2', '')
                        imu_data['acceleration'] = list(map(float, accel_str.split(',')))
                    elif part.startswith("Gyro:"):
                        gyro_str = part.split(':')[-1].strip().replace(']', '').replace('[', '')
                        gyro_str = gyro_str.replace('rad/s', '')
                        imu_data['gyroscope'] = list(map(float, gyro_str.split(',')))

                new_msgs = response.split(",")
                for part in new_msgs:
                    if part.startswith("Heading:"):
                        heading_str = part.split(':')[-1].strip()
                        imu_data['heading'] = float(heading_str)
                    elif part.startswith("Positionx"):
                        posx_str = part.split(":")[-1].strip()
                        imu_data['positionx'] = float(posx_str)
                    elif part.startswith("Positiony:"):
                        posy_str = part.split(":")[-1].strip()
                        imu_data['positiony'] = float(posy_str)
                    elif part.startswith("Speed:"):
                        speed_str = part.split(':')[-1].strip()
                        imu_data['speed'] = float(speed_str)
                        # Add more parts as needed (e.g., magnetometer)

                return imu_data
            except (ValueError, IndexError):
                print(f"[ERROR] Invalid IMU data response: {response}")
        return {}

    def get_imu_angles(self, degrees=True, raw=False) -> dict:
        """
        Computes the orientation of the exo device based on the imu data received

        Args:
            degrees (bool): If True, returns roll, pitch, and yaw in degrees. If False, returns in radians.
            raw (bool): If True, returns raw IMU data instead of computed orientation.

        Returns:
            dict: A dictionary containing orientation data (e.g., roll, pitch, yaw).
        """
        data = self.get_imu_data()
        if data:
            if raw:
                return data

            # compute the roll, pitch, and yaw from the accelerometer and gyroscope data
            accel = data.get('acceleration', [0, 0, 0])
            gyro = data.get('gyroscope', [0, 0, 0])
            return self.imu_filter.update(accel=accel, gyro=gyro)
            # roll = np.atan2(accel[1], accel[2])
            # pitch = np.atan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
            # yaw = np.atan2(gyro[1], gyro[0])

            # if degrees:
            #     roll = np.degrees(roll)
            #     pitch = np.degrees(pitch)
            #     yaw = np.degrees(yaw)

            # return {
            #     'roll': float(roll),
            #     'pitch': float(pitch),
            #     'yaw': float(yaw)
            # }

    def get_imu_heading(self) -> float:     #TODO: add error catch for when none value is parsed by imu_get_data
        data = self.get_imu_data()
        data = self.get_imu_data()


        if data: 
            try:
                heading = data['heading']
                return heading
            except Exception as e:
                print(f"[ERROR] Failed to parse heading from IMU angles: {e}")
        # else:
            # print("No data parsed by get_imu_data()")
        

    def get_gesture_state(self):
        """
        Retrieves the current state of the gesture.

        Returns:
            str: Current gesture state (e.g., "default", "active").

        """
        self.send_command("get_gesture_state")
        response = self._receive()
        if response:
            try:
                return response.split(':')[-1].strip()
            except IndexError:
                print(f"[ERROR] Invalid response")
        return ""