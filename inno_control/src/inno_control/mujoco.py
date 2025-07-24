import mujoco
import mujoco.viewer
import threading
import time
import os


class Simulation:
    """Simulation wrapper for a MuJoCo-based cartpole system.

    This class handles MuJoCo model initialization, actuator control,
    simulation stepping, viewer integration, and external controller callbacks.
    """

    def __init__(self, model_path: str = None):
        """Initializes the Simulation object.

        Args:
            model_path (str, optional): Path to the MuJoCo XML model file. If not provided,
                defaults to "../inno_control/models/cart_pole/cart-pole.xml".
        """
        if not model_path:
            self.model_path = os.path.join(
                os.path.dirname(__file__), '..', 'inno_control', 'models', 'cart_pole', 'cart-pole.xml'
            )
            self.model_path = os.path.abspath(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None  # Viewer window for visualization
        self.running = False  # Flag for simulation loop
        self._lock = threading.Lock()  # Thread lock for safe concurrent access

        self._control_value = 0.0  # Direct control signal (e.g. torque)
        self._custom_controller = None  # Optional user-defined controller function

        # Get joint indices by name for fast lookup
        self._carriage_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "carriage_slide")
        self._pendulum_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "pendulum_hinge")

        # Start simulation loop in background thread
        self._sim_thread = threading.Thread(target=self.run)
        self._sim_thread.start()

    def set_control(self, value: float):
        """Sets the raw actuator control value.

        This method is ignored if a custom controller is registered.

        Args:
            value (float): Control signal (e.g., motor torque or force).
        """
        with self._lock:
            self._control_value = value

    def set_controller(self, fn):
        """Registers a user-defined control function.

        This function will override direct control values.

        Args:
            fn (Callable[[Simulation], None]): A function that accepts the current
                Simulation instance and applies control logic to it.
        """
        self._custom_controller = fn

    def get_state(self) -> tuple[float, float, float, float]:
        """Retrieves the full system state from the simulation.

        Returns:
            tuple: A 4-element tuple containing:
                - x (float): Cart position.
                - x_dot (float): Cart velocity.
                - theta (float): Pendulum angle (in radians).
                - theta_dot (float): Pendulum angular velocity.
        """
        x = self.data.qpos[self._carriage_id]
        x_dot = self.data.qvel[self._carriage_id]
        theta = self.data.qpos[self._pendulum_id]
        theta_dot = self.data.qvel[self._pendulum_id]
        return x, x_dot, theta, theta_dot

    def step(self):
        """Advances the physics simulation by one step."""
        mujoco.mj_step(self.model, self.data)

    def run(self, realtime: bool = True, duration: float = None):
        """Starts the simulation loop with visualization.

        Args:
            realtime (bool, optional): Whether to run the simulation in real time.
                If True, steps are synced to wall-clock time.
            duration (float, optional): If provided, the simulation stops automatically
                after the specified duration (in seconds).
        """
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            self.running = True
            start_time = time.time()

            while viewer.is_running() and self.running:
                with self._lock:
                    if self._custom_controller is not None:
                        self._custom_controller(self)
                    else:
                        self.data.ctrl[0] = self._control_value

                self.step()
                viewer.sync()

                if duration is not None and (time.time() - start_time) > duration:
                    break

    def stop(self):
        """Signals the simulation loop to terminate on the next iteration."""
        self.running = False
