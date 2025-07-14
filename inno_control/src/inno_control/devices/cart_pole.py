from inno_control.devices import LabDevice
from inno_control.exceptions import DeviceConfigurationError, DeviceCommandError
from typing import Optional
from time import sleep




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
            
            self._restart_device()

            while self._connection.in_waiting:
                self._flush()
            
            self._check_response(self._send_command("MOTOR_INIT", read_response=True), 'Initializing motor')
            print('Waiting for initialization of CartPole')
            self._check_response(self._read(sync=True), 'Initialize ended')

            print('CartPole is ready for work')
            self._state = "READY"
            
        except (ValueError, DeviceCommandError) as e:
            raise DeviceConfigurationError(f"Initialization failed: {str(e)}") from e
        




    def _check_response(self, response, expected_response) -> None:
        if not response:
            raise DeviceCommandError('No response')
        
        if response.split()[3:] != expected_response.split():
            raise DeviceCommandError(f'Not expected response {response}')
        

    def _restart_device(self) -> None:
        self._send_command("1000003")
        sleep(2)
        if self._connection.in_waiting:
            self._flush()
        sleep(0.1)
        if self._connection.in_waiting:
            self._restart_device()





    def start_experimnet(self) -> None:
        """Starting experement"""
        self._check_response(self._send_command("START_OPER", read_response=True), 'Starting operational state')

        self._state = "OPER"
    

    def get_state(self):
        return self._state
    

    def get_joint_state(self) -> Optional[str]:
        if self._state != "OPER":
            raise DeviceCommandError("Wrong state of the system, need to switch to 'OPER'")
            
        if self._connection.in_waiting:
            response = self._read()
            if self._connection.in_waiting > 100:
                print(f'slow on {self._connection.in_waiting} bytes, flushing i/o buffers')
                self._flush()
                pass
            return response
        else:
            return None
        
    

    def stop_experiment(self) -> None:
        if self._state == "OPER": 
            self._send_command("1000001")
            print('Stoping...')

                
    def restart(self) -> None:     
        if self._state == "OPER": 
            self._send_command("1000001")
            print('Stoping...')
            self._state = "READY"
        elif self._state == "READY":
            self._send_command("RESTART")
            print(self._read())
            self._state = "READY"




    def set_joint_efforts(self, effort: str) -> None:
        if self._state != "OPER":
            raise DeviceCommandError("Wrong state of the system, need to switch to 'OPER'")
        self._send_command(effort)



    def stop_motor(self) -> None:
        if self._state == "OPER": 
            self._send_command("1000000")
            print('Stoping motor...')
            self._state == "READY"


    def help_me(self) -> None:
        if self._state == "READY": 
            self._send_command("HELP")
        else: 
            raise DeviceCommandError("Wrong state of the system, need to switch to 'READY'")

