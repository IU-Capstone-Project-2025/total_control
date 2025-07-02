import serial
import threading
import queue
import time
from collections import deque 
from typing import Optional
from ..exceptions import DeviceConnectionError, DeviceCommandError

class LabDevice:
    """Base class for laboratory equipment communication via serial interface"""
    
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
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
        self.buffer_input = deque() 
        self.buffer_output = deque()   
        self.lock = threading.Lock()  
        #File to save data fron esp

        
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
        self.buffer_output.append(command)
    
    def _send_command(self, encoding: str = 'utf-8') -> Optional[str]:
        """
        Send command to device 

        Args:
            command: Command string to send
            encoding: Text encoding to use ('utf-8')
            
        Raises:
            DeviceCommandError: If command fails to execute
        """
        while True:
            if not self._connection or not self._connection.is_open:
                raise DeviceConnectionError("No active device connection")
        
            #with self.lock:
                #if not self.buffer:
                    #raise IndexError("No commands in buffer to send")
            if self.buffer_output:
                try:
                    self._connection.write(f"{self.buffer_output.pop()}".encode(encoding))
                except serial.SerialException as e:
                    raise DeviceCommandError(f"Command execution failed: {str(e)}")
        

    
    def _read(self, encoding: str = 'utf-8') -> str:
        while True:
            if self._connection.in_waiting:
                data = self._connection.readline().decode(encoding)
                self.buffer_input.append(data)


    def read_from_buff(self):
        if not self.buffer_input:
            return False
        elements = self.buffer_input.pop().split()  
        return elements[0], elements[2]       
     
    
    def __enter__(self):
        """Context manager entry point"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit point"""
        self.disconnect()



    def start_read_write(self):

        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._send_command, daemon=True).start()
        
        # try:
        #     while True:
        #         user_input = input("print (exit) to finish program or force ")
        #         if user_input.lower() == 'exit':
        #             break
        #         self.buffer_output.append(user_input)
        # except KeyboardInterrupt:
        #     print("program finished ")
        # finally:
        #     if self._connection:
        #         self._connection.close()