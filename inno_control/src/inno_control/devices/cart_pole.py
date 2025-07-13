from inno_control.devices import LabDevice
from inno_control.exceptions import DeviceConfigurationError, DeviceCommandError
import time
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

    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 1.0):
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
        """
        Begin the balancing experiment.

        This method puts the system into active balancing mode, where the motor controller
        will apply control efforts to keep the pendulum upright.

        Raises:
            DeviceCommandError: If the system does not confirm that balancing has started.
        """
        response = self._send_command("START_OPER", read_response=True)
        
        if response != "STARTED":
            raise DeviceCommandError(response)
        
        self._state = "STARTED"
    
    def get_state(self):
        """
        Get the current state of the Cart-Pole device.

        Returns:
            str: Current system state: 'UNKNOWN', 'READY', or 'STARTED'.
        """
        return self._state
    
    def get_joint_state(self) -> None:
        """
        Read the current physical state of the cart and pole.

        When the experiment is running, this reads data such as the cart position,
        velocity, pendulum angle, and angular velocity from the onboard sensors.

        Returns:
            str: Raw sensor data as received from the ESP32.

        Raises:
            DeviceCommandError: If called when the system is not running.
        """
        if self._state == "STARTED":
            return self._read()
        raise DeviceCommandError("Wrong state of the system, need to switch to 'STARTED'")
    
    def stop_experiment(self) -> None:
        """
        Stop the balancing experiment and switch the system back to idle.

        This sends a command to stop applying control forces and returns the system
        to a safe idle state. It verifies that the system acknowledges the mode change.

        Raises:
            DeviceCommandError: If the system fails to return to 'READY' mode.
        """
        if self._state == "STARTED":
            
            self._send_command("MODE=READY", read_response=True)
            
            print('Stoping...')
            time.sleep(2.0)
            response = self._send_command("STATE", read_response_in=True)
            if response != "READY":
                raise DeviceCommandError(f"Device not responding properly.\nState: {response}")
            
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
        self._send_command(effort)
