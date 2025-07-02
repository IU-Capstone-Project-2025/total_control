from inno_control.devices import LabDevice
from inno_control.exceptions import DeviceConfigurationError, DeviceCommandError
import time
import threading

class CartPole(LabDevice):
    """
    Class for entire Cart-Pole work
    """

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
        super().__init__(port, baudrate, timeout)
        self._state = "UNKNOWN"
        

    def _initialize_device(self) -> None:
        """Initializes the CartPole device.

        Performs the following steps:
        1. Sends MOTOR_INIT command to initialize motors
        2. Starts parallel process 
        3. Sets device state to READY if successful

        Raises:
            DeviceConfigurationError: If initialization fails due to:
                - ValueError: Invalid parameter or value encountered
                - DeviceCommandError: Command to device failed

        Note:
            This is an internal method and should not be called directly by users.
        """
        try:
            self.add_command("RESET")
            self.add_command("1000003")
            self.add_command("MOTOR_INIT")

            self.start_read_write()
            
            self._state = "READY"
            
        except (ValueError, DeviceCommandError) as e:
            raise DeviceConfigurationError(f"Initialization failed: {str(e)}") from e

    def start_experimnet(self) -> None:
        """Starting experement"""
        self.add_command("START_OPER")
        print("xyz")
        
        self._state = "STARTED"
    
    def get_joint_state(self) -> None:
        if self._state == "STARTED":
            resp = self.read_from_buff()
            if not resp:
                return
            return resp
        raise DeviceCommandError("Wrong state of the system, need to switch to 'STARTED'")
    
    def stop_experiment(self) -> None:
        if self._state == "STARTED":
            
            self.add_command("MODE=READY")
            
            print('Stoping...')
            time.sleep(2.0)
            self.add_command("STATE")
            
    def set_joint_efforts(self, effort: str) -> None:
        
        self.add_command(effort)
