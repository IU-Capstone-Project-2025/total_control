Simulation guide
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

Downloading MuJoCo (Linux)
~~~~~~~~~~~~~~~~~

1. Visit the official site: https://mujoco.org/download
2. Download the appropriate version for your OS (Linux)
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

Testing
------------

Create `test_mujoco.py`:

.. code-block:: python

   import mujoco
   import os
   
   # Path to the model (change it to suit your system)
   model_path = os.path.expanduser('<path_to_model>')
   
   model = mujoco.MjModel.from_xml_path(model_path)
   data = mujoco.MjData(model)
   
   for _ in range(1000):
       mujoco.mj_step(model, data)
       print(data.qpos)

Run the test:

.. code-block:: bash

   python test_mujoco.py

Troubleshooting
---------------

1. License errors:
   - Ensure `mjkey.txt` exists in `~/.mujoco/`

2. GLFW issues:
   .. code-block:: bash
      sudo apt install libglfw3 libglfw3-dev

3. Video driver problems:
   - Update your OpenGL drivers

4. Most of the problems with mujoco py lib can be soleved by suggestions from lib error

Additional Resources
-------------------

- Official docs: https://mujoco.readthedocs.io
- Examples: https://github.com/deepmind/mujoco
- Forum: https://github.com/deepmind/mujoco/discussions


MuJoCo simulation
-----------------

.. autoclass:: inno_control.mujoco
   :members:
   :undoc-members:
   :show-inheritance: