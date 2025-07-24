from inno_control.devices import LabDevice
from inno_control.exceptions import DeviceConfigurationError, DeviceCommandError
from typing import Optional
from time import sleep
import threading
from inno_control.mujoco import Simulation
import os


class CartPole(LabDevice):
    """
    Interface for controlling and reading from a physical Cart-Pole system via an ESP32 device.

    This class allows you to initialize, start, stop, and control a real inverted pendulum on a cart.
    It communicates via serial port, sending commands to the onboard controller which drives the cart motor
    to balance the pendulum upright by applying horizontal forces.

    The Cart-Pole is a classic non-linear control benchmark: the goal is to keep the pendulum in unstable equilibrium
    by continuously adjusting the cart position.

    Attributes:
        _state (str): Current state of the system, can be 'UNKNOWN', 'READY', or 'STARTED'.
    """

    def __init__(self, port: str="SIM", baudrate: int = 921600, timeout: float = 1.0):
        """
        Create a new CartPole device interface.

        Opens a serial connection to the ESP32 device that controls the cart motor and reads sensor data.

        Args:
            port (str): Serial port (e.g., '/dev/ttyUSB0', 'COM3').
            baudrate (int, optional): Serial communication speed in baud. Defaults to 921600.
            timeout (float, optional): Timeout for serial reads/writes in seconds. Defaults to 1.0.
        """
        super().__init__(port, baudrate, timeout)
        self._state = "UNKNOWN"
        

    def _initialize_device(self) -> None:
        """
        Initialize the Cart-Pole hardware by sending motor setup commands.

        This sends an initialization command to prepare the motor controller and sensors.
        During this step, the device may perform self-checks or calibrations.

        Raises:
            DeviceConfigurationError: If the device fails to initialize properly.
        """
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
        
    def start_simulation(self):
        """Start simulation in MuJoCo"""
        model_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'cart_pole', 'cart-pole.xml')
        model_path = os.path.abspath(model_path)
        self._connection = Simulation(model_path)
        self._sim_thread = threading.Thread(target=self._connection.run)
        self._sim_thread.start()

    def delete(self):
        return self._connection

    def re_init(self):
        """TODO"""
        self._initialize_device()
        self.start_experimnet()

    def _check_response(self, response, expected_response) -> None:
        """TODO"""
        if not response:
            raise DeviceCommandError('No response')
        
        if response.split()[3:] != expected_response.split():
            raise DeviceCommandError(f'Not expected response {response}')
        

    def _restart_device(self) -> None:
        """
        TODO
        """
        self._send_command("1000003")
        sleep(2)
        if self._connection.in_waiting:
            self._flush()
        sleep(0.1)
        if self._connection.in_waiting:
            self._restart_device()





    def start_experimnet(self) -> None:
        """
        Begin the balancing experiment.

        This method puts the system into active balancing mode, where the motor controller
        will apply control efforts to keep the pendulum upright.

        Raises:
            DeviceCommandError: If the system does not confirm that balancing has started.
        """
        self._check_response(self._send_command("START_OPER", read_response=True), 'Starting operational state')

        self._state = "OPER"
    

    def get_state(self):
        """
        Get the current state of the Cart-Pole device.

        Returns:
            str: Current system state: 'UNKNOWN', 'READY', or 'STARTED'.
        """
        return self._state
    

    def get_joint_state(self) -> Optional[str]:
        """
        Read the current physical state of the cart and pole.

        When the experiment is running, this reads data such as the cart position,
        velocity, pendulum angle, and angular velocity from the onboard sensors.

        Returns:
            str: Raw sensor data as received from the ESP32.

        Raises:
            DeviceCommandError: If called when the system is not running.
        """
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
        """
        Stop the balancing experiment and switch the system back to idle.

        This sends a command to stop applying control forces and returns the system
        to a safe idle state. It verifies that the system acknowledges the mode change.

        Raises:
            DeviceCommandError: If the system fails to return to 'READY' mode.
        """
        if self._state == "OPER": 
            self._send_command("1000001")
            print('Stoping...')

                
    def _restart(self) -> None:
        """TODO"""     
        if self._state == "OPER": 
            self._send_command("1000001")
            print('Stoping...')
            self._state = "READY"
        elif self._state == "READY":
            self._send_command("RESTART")
            print(self._read())
            self._state = "READY"




    def set_joint_efforts(self, effort: str) -> None:
        """
        Send a control effort command to the cart motor.

        This lets you directly set the motor control effort or apply a specific force,
        for example, to test responses or run custom controllers.

        Args:
            effort (str): Effort command string (e.g., 'EFFORT=0.2'). The format must match
                what the device firmware expects.

        Raises:
            DeviceCommandError: If the effort command cannot be sent.
        """
        if self._state != "OPER":
            raise DeviceCommandError("Wrong state of the system, need to switch to 'OPER'")
        self._send_command(effort)



    def stop_motor(self) -> None:
        """TODO"""
        if self._state == "OPER": 
            self._send_command("1000000")
            print('Stoping motor...')
            self._state == "READY"


    def help_me(self) -> None:
        """TODO"""
        if self._state == "READY": 
            self._send_command("HELP")
        else: 
            raise DeviceCommandError("Wrong state of the system, need to switch to 'READY'")

