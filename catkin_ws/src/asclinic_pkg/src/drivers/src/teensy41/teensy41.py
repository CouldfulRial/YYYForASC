import serial


class Teensy41:
    """
    This class is an abstraction to interact with the Teensy to retrieve
    information from the differential encoder.
    """

    def __init__(self, device_name="/dev/ttyACM0", timeout=0):
        self._serial_port = serial.Serial(device_name, timeout=timeout)

    def close_device(self):
        """
        Closes the device.
        """
        if self._serial_port.is_open:
            self._serial_port.close()

    def get_motors_displacement(self):
        """
        Gets motors' displacement without timestamp.
        """
        self._serial_port.write(b"*")
        while len(string_from_teensy) == 0:
            # 50 characters should be enough, but if not, just increase this
            string_from_teensy = self._serial_port.read(50)
        strings_with_braces = string_from_teensy.decode().split(",")
        if strings_with_braces[0][0] != "<" or strings_with_braces[1][-1] != ">":
            error_message = "The data from teensy is corrupted."
            raise Exception(error_message)
        disp1 = int(strings_with_braces[0][1:])
        disp2 = int(strings_with_braces[1][:-1])
        return (disp1, disp2)

    def get_motors_displacement_with_timestamp(self):
        """
        Gets motors' displacemment with timestamp.
        """
        self._serial_port.write(b"t")
        string_from_teensy = ""
        while len(string_from_teensy) == 0:
            # 50 characters should be enough, but if not, just increase this
            string_from_teensy = self._serial_port.read(50)
        strings_with_braces = string_from_teensy.decode().split(",")
        if strings_with_braces[0][0] != "{" or strings_with_braces[-1][-1] != "}":
            error_message = "The data from teensy is corrupted."
            raise Exception(error_message)
        disp1 = int(strings_with_braces[0][1:])
        disp2 = int(strings_with_braces[1])
        timestamp = int(strings_with_braces[-1][:-1])
        return (disp1, disp2, timestamp)

    def get_motors_speed_rpm(self):
        """
        Gets motors' speed in RPM.
        """
        self._serial_port.write(b"s")
        while len(string_from_teensy) == 0:
            # 50 characters should be enough, but if not, just increase this
            string_from_teensy = self._serial_port.read(50)
        strings_with_braces = string_from_teensy.decode().split(",")
        if strings_with_braces[0][0] != "[" or strings_with_braces[1][-1] != "]":
            error_message = "The data from teensy is corrupted."
            raise Exception(error_message)
        speed1 = int(strings_with_braces[0][1:])
        speed2 = int(strings_with_braces[1][:-1])
        return (speed1, speed2)

    @property
    def device_name(self):
        """
        Returns the device's name.
        """
        return self._serial_port.name
