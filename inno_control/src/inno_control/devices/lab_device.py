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

        Attributes initialized:
            _port (str): Stores the serial port name
             _baudrate (int): Stores communication speed
            _timeout (float): Stores read timeout value
            _connection: Will hold the serial connection object (initialized as None)
            buffer (deque): Thread-safe buffer for incoming data
            lock (threading.Lock): Synchronization primitive for thread-safe operations
            encoder_log (str): Filename for storing encoder data logs
        """
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._connection = None
        self.buffer = deque()  
        self.lock = threading.Lock()  
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


    def add_command(self, command: str):
        """Adds a command to the thread-safe command queue.

            This method appends a command to the internal buffer for subsequent processing.
            The operation is thread-safe using a lock to prevent concurrent access issues.

        Args:
            command: The command string to be added to the queue. Should be a valid
            device command according to the protocol specifications.
        """
        with self.lock:
            self.buffer.append(command)
    
    def _send_command(self, encoding: str = 'utf-8') -> Optional[str]:
        """
        Send command to device 

        Args:
            command: Command string to send
            encoding: Text encoding to use ('utf-8')
            
        Raises:
            DeviceCommandError: If command fails to execute
        """
        if not self._connection or not self._connection.is_open:
            raise DeviceConnectionError("No active device connection")
        
        with self.lock:
            if not self.buffer:
                raise IndexError("No commands in buffer to send")

        try:
            self._connection.write(f"{self.buffer}".encode(encoding))
            self.buffer = ""
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

    def read_from_buff(self, number):
        elements = self.buffer[:number]    
        self.buffer = self.buffer[number:] # ??? Do we need this row?
        return elements        

                
    
    def __enter__(self):
        """Context manager entry point"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point"""
        self.disconnect()


    

    def start_read_write(self, buff_to_send):

        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._send_command, daemon=True).start()
        
        try:
            while True:
                user_input = input("print (exit) to finish program")
                if user_input.lower() == 'exit':
                    break
                self.command_queue.put(user_input)
        except KeyboardInterrupt:
            print("program finished ")
        finally:
            if self.serial_connection:
                self.serial_connection.close()