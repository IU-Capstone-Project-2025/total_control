import serial
import threading
import queue
import time
from collections import deque 
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

        #File to save data fron esp
        self.encoder_log = "encoder_data.log"
        
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
        
    def log_to_file(self, filename, messege):
        """Save messenge to log file"""
        with open(filename, "a") as f:
            f.write(f"{messege}\n")  

    
    def _read(self, encoding: str = 'utf-8') -> str:
        ser = self._connection
        while self.running:
            data = ser.readline().decode()
            if data:
                self.buffer.extend(data)
                self.log_to_file(self.encoder_log, data)

                
    
    def __enter__(self):
        """Context manager entry point"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point"""
        self.disconnect()


    "Я НИЧЕГо НЕ ДОДЕЛАЛА не трогайте "

    def start(self):

        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._send_command, daemon=True).start()

        ""
        
        try:
            while True:
                user_input = input("Введите команду (или 'exit' для выхода): ")
                if user_input.lower() == 'exit':
                    break
                self.command_queue.put(user_input)
        except KeyboardInterrupt:
            print("Программа завершена")
        finally:
            if self.serial_connection:
                self.serial_connection.close()