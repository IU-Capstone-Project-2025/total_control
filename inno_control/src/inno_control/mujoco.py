import mujoco
import mujoco.viewer
import threading
import time
import os


class Simulation:
    """Simulation wrapper for a MuJoCo-based cartpole system.

    This class handles MuJoCo model initialization, actuator control,
    simulation stepping, viewer integration, and external controller callbacks.

    Attributes:
        model (mujoco.MjModel): MuJoCo model object.
        data (mujoco.MjData): MuJoCo simulation data object.
        viewer: Visualization window handle (when active).
        running (bool): Flag indicating if simulation loop is active.
    """


    def __init__(self, model_path: str = None):
        """Initializes the Simulation object.

        Loads the MuJoCo model, initializes simulation data, prepares joint lookups,
        and starts the simulation thread.

        Args:
            model_path: Path to MuJoCo XML model file. Defaults to:
                "../inno_control/models/cart_pole/cart-pole.xml" relative to this file.
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

        Note: Has no effect if a custom controller is registered.

        Args:
            value: Control signal (e.g., motor torque or force).
        """
        with self._lock:
            self._control_value = value

    def set_controller(self, fn):
        """Registers a user-defined control function.

        This function will override direct control values set by `set_control()`.

        Args:
            fn: Callable accepting the Simulation instance as argument.
                Expected signature: `controller(sim: Simulation) -> None`
        """
        self._custom_controller = fn

    def get_state(self) -> tuple[float, float, float, float]:
        """Retrieves the full system state from the simulation.

        Returns:
            Tuple containing four float values:
            - x: Cart position [m]
            - x_dot: Cart velocity [m/s]
            - theta: Pendulum angle [rad]
            - theta_dot: Pendulum angular velocity [rad/s]
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
        """Main simulation loop with visualization.

        Runs until stopped explicitly or until duration expires. Applies control
        signals, steps physics, and updates visualization.

        Args:
            realtime: Attempt to maintain real-time synchronization (not guaranteed).
            duration: Automatic shutdown duration in seconds (optional).
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
