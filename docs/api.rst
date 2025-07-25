API Reference
=============

This API reference provides detailed information about the classes and methods available
in the `inno_control` library.

The library is designed for communicating with laboratory control systems and physical devices
such as the classic Cart-Pole benchmark system. It provides:

- **LabDevice** — a base class for serial device communication.
- **CartPole** — a specific implementation for an inverted pendulum on a cart.

Below are the available classes with their methods and detailed docstrings.


LabDevice
----------------

.. autoclass:: inno_control.devices.LabDevice
   :members:
   :undoc-members:
   :show-inheritance:

Cart-Pole System
----------------

The **Cart-Pole** is a classic non-linear control benchmark. It models an inverted pendulum mounted
on a motorized cart that can move horizontally. The control objective is to apply forces to the cart
so that the pendulum remains upright.

The equations of motion are derived from Newton's second law and the Lagrangian method:

.. math::

   (M + m) \ddot{x} + m l \ddot{\theta} \cos \theta - m l \dot{\theta}^2 \sin \theta = F

.. math::

   I \ddot{\theta} + m l \ddot{x} \cos \theta + m g l \sin \theta = 0

where:

- :math:`x` is the horizontal position of the cart,
- :math:`\theta` is the angle of the pendulum from vertical,
- :math:`M` is the mass of the cart,
- :math:`m` is the mass of the pendulum,
- :math:`l` is the distance to the pendulum center of mass,
- :math:`I` is the moment of inertia of the pendulum about its center of mass,
- :math:`F` is the horizontal force applied to the cart,
- :math:`g` is the acceleration due to gravity.

The Cart-Pole system is inherently unstable and is a popular testbed for advanced control algorithms
such as LQR, swing-up control, and reinforcement learning.

.. autoclass:: inno_control.devices.CartPole
   :members:
   :undoc-members:
   :show-inheritance:

Exceptions
----------
.. autoclass:: inno_control.exceptions
   :members:
   :undoc-members:
   :show-inheritance: