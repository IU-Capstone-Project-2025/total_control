MuJoCo Installation and Python Setup Guide
=========================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Introduction
------------
MuJoCo (Multi-Joint dynamics with Contact) is a physics engine for robotic simulation. This guide covers:

1. Downloading and installing MuJoCo
2. Setting up Python bindings
3. Running a basic example

Installation
------------

Downloading MuJoCo
~~~~~~~~~~~~~~~~~

1. Visit the official site: https://mujoco.org/download
2. Download the appropriate version for your OS (Linux/Windows/macOS)
3. Extract to your home directory:

.. code-block:: bash

   mkdir ~/.mujoco
   tar -xvzf mujoco210-linux-x86_64.tar.gz -C ~/.mujoco

Environment Variables
~~~~~~~~~~~~~~~~~~~~

Add to your `~/.bashrc` or `~/.zshrc`:

.. code-block:: bash

   export MUJOCO_PY_MUJOCO_PATH=~/.mujoco/mujoco210/bin
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco210/bin
   export MUJOCO_KEY_PATH=~/.mujoco/mjkey.txt

Then reload:

.. code-block:: bash

   source ~/.bashrc

Python Package Installation
--------------------------

Recommended (MuJoCo 2.3.0+):

.. code-block:: bash

   pip install mujoco

For legacy versions:

.. code-block:: bash

   pip install mujoco-py

Linux dependencies (Ubuntu/Debian):

.. code-block:: bash

   sudo apt install libosmesa6-dev libgl1-mesa-glx patchelf

Verification
------------

Create `test_mujoco.py`:

.. code-block:: python

   import mujoco
   import mujoco.viewer
   import numpy as np

   model = mujoco.MjModel.from_xml_string("""
   <mujoco>
     <worldbody>
       <geom name="floor" type="plane" size="5 5 0.1"/>
       <body name="box" pos="0 0 1">
         <joint type="free"/>
         <geom name="box" type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
       </body>
     </worldbody>
   </mujoco>
   """)

   data = mujoco.MjData(model)

   with mujoco.viewer.launch_passive(model, data) as viewer:
       while viewer.is_running():
           mujoco.mj_step(model, data)
           viewer.sync()

Run the test:

.. code-block:: bash

   python test_mujoco.py

Example: Cartpole Model
-----------------------

Create `cartpole.xml`:

.. literalinclude:: cartpole.xml
   :language: xml
   :caption: cartpole.xml

Run with this script:

.. code-block:: python

   import mujoco
   import mujoco.viewer
   import numpy as np

   model = mujoco.MjModel.from_xml_path("cartpole.xml")
   data = mujoco.MjData(model)

   # Initialization
   data.qpos[1] = 0  # Cart position
   data.qpos[2] = np.pi  # Pendulum angle

   with mujoco.viewer.launch_passive(model, data) as viewer:
       while viewer.is_running():
           # Random control inputs
           data.ctrl[0] = 0.1 * np.random.randn()
           mujoco.mj_step(model, data)
           viewer.sync()

Troubleshooting
---------------

1. License errors:
   - Ensure `mjkey.txt` exists in `~/.mujoco/`

2. GLFW issues:
   .. code-block:: bash
      sudo apt install libglfw3 libglfw3-dev

3. Video driver problems:
   - Update your OpenGL drivers

Additional Resources
-------------------

- Official docs: https://mujoco.readthedocs.io
- Examples: https://github.com/deepmind/mujoco
- Forum: https://github.com/deepmind/mujoco/discussions
