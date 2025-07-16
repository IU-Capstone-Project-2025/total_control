import serial
import time

class HardwareController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.ser.flush()
        self.ready()

    def ready(self):
        while True:
            line = self.ser.readline().decode().strip()
            if "Ready to use" in line:
                break

    def send_command(self, cmd):
        self.ser.write((str(cmd) + "\n").encode())

    def get_joint_state(self):
        self.send_command("1000002")
        raw = self.ser.readline().decode().strip()
        try:
            values = list(map(float, raw.split()))
            return values
        except:
            return None

    def set_joint_efforts(self, effort):
        effort = int(max(-32000, min(32000, effort)))
        self.send_command(str(effort))

    def stop(self):
        self.send_command("1000000")  # Emergency stop

    def shutdown(self):
        self.send_command("1000001")  # Go to Ready state
        self.ser.close()
