import serial
from typing import Optional
from ..exceptions import DeviceConnectionError, DeviceCommandError

class LabDevice:
    """Base class for laboratory equipment communication via serial interface"""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize the lab device connection
        
        Args:
            port: Serial port name (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
            baudrate: Communication speed in bits per second (default: 9600)
            timeout: Read timeout in seconds (default: 1.0)
        """
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._connection = None
        
    def connect(self) -> None:
        """Establish connection with the lab device"""
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
        """Safely close the device connection"""
        if self._connection and self._connection.is_open:
            self._connection.close()
        self._connection = None
    
    def _initialize_device(self) -> None:
        """Device-specific initialization (override in child classes)"""
        pass
    
    def _send_command(self, command: str, read_response: bool = False, 
                    encoding: str = 'utf-8') -> Optional[str]:
        """
        Send command to device and optionally read response
        
        Args:
            command: Command string to send
            read_response: Whether to wait for response (default: True)
            encoding: Text encoding to use (default: 'ascii')
            
        Returns:
            Device response as string if read_response=True, None otherwise
            
        Raises:
            DeviceCommandError: If command fails to execute
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
        Read response from device
        
        Args:
            encoding: Text encoding to use (default: 'ascii')
            
        Returns:
            Decoded response string
            
        Raises:
            DeviceCommandError: If read operation fails
        """
        try:
            return self._connection.readline().decode(encoding).strip()
        except serial.SerialException as e:
            raise DeviceCommandError(f"Failed to read response: {str(e)}")
    
    def __enter__(self):
        """Context manager entry point"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point"""
        self.disconnect()