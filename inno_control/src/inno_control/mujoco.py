import mujoco
import mujoco.viewer
import threading
import time
import os

class Simulation:
    """Simulation wrapper for a MuJoCo-based cartpole system.

    Handles model loading, actuator control, simulation stepping,
    and state observation.
    """

    def __init__(self, model_path: str=None):
        """Initialize the simulation with the given MuJoCo XML model.

        Args:
            model_path (str): Path to the XML model file.
        """

        if not model_path:
            self.model_path = os.path.join(os.path.dirname(__file__), '..', 'inno_control', 'models', 'cart_pole', 'cart-pole.xml')
            self.model_path = os.path.abspath(self.model_path)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        self.running = False
        self._lock = threading.Lock()

        self._control_value = 0.0
        self._custom_controller = None

        # Resolve joint IDs from joint names
        self._carriage_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "carriage_slide")
        self._pendulum_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "pendulum_hinge")

        self._sim_thread = threading.Thread(target=self.run)
        self._sim_thread.start()

    def set_control(self, value: float):
        """Apply direct control signal to actuator (overridden if custom controller set).

        Args:
            value (float): Control effort (e.g. torque).
        """
        with self._lock:
            self._control_value = value

    def set_controller(self, fn):
        """Set a user-defined control callback function.

        Args:
            fn (callable): Function that accepts a `Simulation` instance.
        """
        self._custom_controller = fn

    def get_state(self) -> tuple[float, float, float, float]:
        """Return full system state.

        Returns:
            tuple: (x, x_dot, theta, theta_dot)
              - x: cart position
              - x_dot: cart velocity
              - theta: pendulum angle (radians)
              - theta_dot: pendulum angular velocity
        """
        x = self.data.qpos[self._carriage_id]
        x_dot = self.data.qvel[self._carriage_id]
        theta = self.data.qpos[self._pendulum_id]
        theta_dot = self.data.qvel[self._pendulum_id]
        return x, x_dot, theta, theta_dot

    def step(self):
        """Advance the simulation by one time step."""
        mujoco.mj_step(self.model, self.data)

    def run(self, realtime: bool = True, duration: float = None):
        """Launch simulation loop and optionally control in real-time.

        Args:
            realtime (bool): If True, sync simulation to wall time.
            duration (float, optional): Time to run before auto-stopping.
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
        """Signal the simulation loop to terminate."""
        self.running = False
