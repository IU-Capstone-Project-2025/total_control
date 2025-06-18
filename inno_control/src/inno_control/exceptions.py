class LabDeviceError(Exception):
    """
    Base exception for all lab equipment related errors.
    
    Attributes:
        device (str): Name or identifier of the device that caused the error
        message (str): Human-readable error description
        original_exception (Exception, optional): Original exception if this is a wrapper
    """
    
    def __init__(self, message: str, device: str = None, original_exception: Exception = None):
        """
        Initialize base lab equipment error.
        
        Args:
            message: Error description
            device: Device name/identifier (default: None)
            original_exception: Original exception (default: None)
        """
        self.device = device
        self.message = message
        self.original_exception = original_exception
        full_msg = f"Device error: {message}"
        if device:
            full_msg = f"[{device}] {full_msg}"
        if original_exception:
            full_msg = f"{full_msg} (Original: {str(original_exception)})"
        super().__init__(full_msg)


class DeviceConnectionError(LabDeviceError):
    """
    Device connection error.
    
    Raised for connection establishment, maintenance or termination issues.
    
    Attributes:
        port (str): Connection port attempted
        baudrate (int): Connection speed
        connection_type (str): Type of connection (serial, ethernet etc.)
    """
    
    def __init__(self, message: str, device: str = None, 
                 port: str = None, baudrate: int = None, 
                 connection_type: str = 'serial', original_exception: Exception = None):
        """
        Initialize connection error.
        
        Args:
            message: Error description
            device: Device name
            port: Connection port used
            baudrate: Connection speed
            connection_type: Type of connection
            original_exception: Original exception
        """
        self.port = port
        self.baudrate = baudrate
        self.connection_type = connection_type
        details = []
        if port:
            details.append(f"port={port}")
        if baudrate:
            details.append(f"baudrate={baudrate}")
        if details:
            message = f"{message} ({', '.join(details)}, type={connection_type})"
        super().__init__(message, device, original_exception)


class DeviceCommandError(LabDeviceError):
    """
    Device command execution error.
    
    Raised when command execution fails or invalid response received.
    
    Attributes:
        command (str): Command that caused the error
        response (str): Device response (if any)
        timeout (float): Operation timeout (if applicable)
    """
    
    def __init__(self, message: str, device: str = None, 
                 command: str = None, response: str = None,
                 timeout: float = None, original_exception: Exception = None):
        """
        Initialize command execution error.
        
        Args:
            message: Error description
            device: Device name
            command: Problematic command
            response: Device response
            timeout: Operation timeout
            original_exception: Original exception
        """
        self.command = command
        self.response = response
        self.timeout = timeout
        details = []
        if command:
            details.append(f"command='{command}'")
        if response:
            details.append(f"response='{response}'")
        if timeout:
            details.append(f"timeout={timeout}s")
        if details:
            message = f"{message} ({', '.join(details)})"
        super().__init__(message, device, original_exception)


class DeviceConfigurationError(LabDeviceError):
    """
    Device configuration error.
    
    Raised for invalid configuration parameters or settings.
    
    Attributes:
        parameter (str): Configuration parameter that caused the error
        value (any): Invalid parameter value
        valid_range (tuple): Valid range/values for the parameter (if applicable)
    """
    
    def __init__(self, message: str, device: str = None,
                 parameter: str = None, value: any = None,
                 valid_range: tuple = None, original_exception: Exception = None):
        """
        Initialize configuration error.
        
        Args:
            message: Error description
            device: Device name
            parameter: Problematic parameter
            value: Invalid value provided
            valid_range: Valid range/values for the parameter
            original_exception: Original exception
        """
        self.parameter = parameter
        self.value = value
        self.valid_range = valid_range
        details = []
        if parameter:
            details.append(f"parameter='{parameter}'")
        if value is not None:
            details.append(f"value={value}")
        if valid_range:
            details.append(f"valid_range={valid_range}")
        if details:
            message = f"{message} ({', '.join(details)})"
        super().__init__(message, device, original_exception)