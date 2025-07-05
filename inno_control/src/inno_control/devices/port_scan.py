import serial
import sys
import glob


def scan() -> list:
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def find_your_device() -> str:
    """ Procedure to found a certain serial device

        :raises RuntimeError:
            No device or multiple device found
        :returns:
            single com port in string
    """
    print("\nUplug your device and press enter")
    input()
    ports1 = set(scan())
    print("Plug in your device and press enter again")
    input()
    ports2 = set(scan())
    device = list(ports2 - ports1)

    if len(device) == 0:
        raise RuntimeError("No device found")
    elif len(device) > 1:
        raise RuntimeError(f"Multiple device found {device}")

    print(f"Your device on port {device[0]}")
    return device[0]
