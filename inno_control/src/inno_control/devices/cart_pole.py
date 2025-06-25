from inno_control.devices import LabDevice
from inno_control.exceptions import DeviceConfigurationError, DeviceCommandError
import time
class CartPole(LabDevice):
    """
    Class for entire Cart-Pole work
    """

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
        super().__init__(port, baudrate, timeout)
        self._state = "UNKNOWN"
        

    def _initialize_device(self) -> None:
        """Initialization of CartPole"""
        try:
            self._send_command("MOTOR_INIT")
            
            print('Waiting for initialization of CartPole')
            print(self._read())
            print(self._read())
            print(self._read())
            print(self._read())
            self._state = "READY"
            
        except (ValueError, DeviceCommandError) as e:
            raise DeviceConfigurationError(f"Initialization failed: {str(e)}") from e

    def start_experimnet(self) -> None:
        """Starting experement"""
        response = self._send_command("MODE=EXP", read_response=True)
        
        if response != "STARTED":
            raise DeviceCommandError(response)
        
        self._state = "STARTED"
    
    def get_joint_state(self) -> None:
        if self._state == "STARTED":
            return self._read()
        raise DeviceCommandError("Wrong state of the system, need to switch to 'STARTED'")
    
    def stop_experiment(self) -> None:
        if self._state == "STARTED":
            
            self._send_command("MODE=READY", read_response=True)
            
            print('Stoping...')
            time.sleep(2.0)
            response = self._send_command("STATE", read_response_in=True)
            if response != "READY":
                raise DeviceCommandError(f"Device not responding properly.\nState: {response}")
            
    def set_joint_efforts(self, effort: str) -> None:
        
        self._send_command(effort)
