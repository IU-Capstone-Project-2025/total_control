import serial
from typing import Optional
from ..exceptions import DeviceConnectionError, DeviceCommandError

class LabDevice:
    """
    Base class for laboratory equipment communication via a serial interface.

    This class defines a generic interface to connect to any lab device (e.g., a motor controller, sensor board, or actuator)
    that communicates using a serial port. It provides core methods for connecting, disconnecting, sending commands,
    and reading responses. Specific devices should extend this base class and implement any custom initialization logic
    in `_initialize_device`.

    Attributes:
        _port (str): Serial port used to connect to the device.
        _baudrate (int): Communication speed in baud.
        _timeout (float): Timeout for serial operations.
        _connection (serial.Serial): Active serial connection.
    """
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Establish the serial connection with the lab device.

        Opens the serial port with the specified configuration and calls
        `_initialize_device` to perform any device-specific setup after connection.

        Raises:
            DeviceConnectionError: If the serial port cannot be opened.
        """
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._connection = None
        
    def connect(self) -> None:
        """
        Close the serial connection safely.

        Closes the serial port if it is open and clears the connection attribute.
        """
        try:
            self._connection = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=self._timeout
            )
            # Device-specific initialization
            self._initialize_device()
            
        except serial.SerialException as e:
            raise DeviceConnectionError(f"Connection to {self._port} failed: {str(e)}")
    
    def disconnect(self) -> None:
        """
        Close the serial connection safely.

        Closes the serial port if it is open and clears the connection attribute.
        """
        if self._connection and self._connection.is_open:
            self._connection.close()
        self._connection = None
    
    def _initialize_device(self) -> None:
        """
        Perform any additional initialization steps for the device.

        This method should be overridden in a child class if the device requires
        any commands to be sent immediately after connecting (e.g., motor setup,
        sensor calibration).
        """
        pass
    
    def _send_command(self, command: str, read_response: bool = False, 
                    encoding: str = 'utf-8') -> Optional[str]:
        """
        Send a command string to the device and optionally read a response.

        Args:
            command (str): The command to send to the device.
            read_response (bool, optional): If True, read and return a single line response. Defaults to False.
            encoding (str, optional): Character encoding for the command and response. Defaults to 'utf-8'.

        Returns:
            Optional[str]: Response string if `read_response` is True, otherwise None.

        Raises:
            DeviceConnectionError: If the device is not connected.
            DeviceCommandError: If the command fails to send or read.
        """
        if not self._connection or not self._connection.is_open:
            raise DeviceConnectionError("No active device connection")
        try:
            self._connection.write(f"{command}\n".encode(encoding))
            if read_response:
                return self._connection.readline().decode(encoding).strip()
            return None
        except serial.SerialException as e:
            raise DeviceCommandError(f"Command execution failed: {str(e)}")
    
    def _read(self, encoding: str = 'utf-8') -> str:
        """
        Read a single line of response from the device.

        Args:
            encoding (str, optional): Character encoding to decode the response. Defaults to 'utf-8'.

        Returns:
            str: Decoded response line from the device.

        Raises:
            DeviceCommandError: If reading the response fails.
        """
        try:
            return self._connection.readline().decode(encoding).strip()
        except serial.SerialException as e:
            raise DeviceCommandError(f"Failed to read response: {str(e)}")
    
    def __enter__(self):
        """
        Enter the context manager, automatically connecting the device.

        Returns:
            LabDevice: The connected device instance.
        """
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit the context manager, automatically disconnecting the device.

        Ensures the serial connection is safely closed even if an exception occurs.
        """
        self.disconnect()