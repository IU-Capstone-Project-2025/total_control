Installation
============

The **innocontrol** library supports multiple installation methods for different use cases.

Prerequisites
-------------
- Python 3.10 or newer
- USB access (for hardware control)
- Supported OS: Linux/macOS/Windows (64-bit)

Standard Installation
--------------------
Install from PyPI:

.. code-block:: bash

   pip install innocontrol

Development Installation
------------------------
For contributing to the project:

1. Clone the repository:

   .. code-block:: bash

      git clone https://github.com/IU-Capstone-Project-2025/total_control.git
      cd total_control

2. Install with development dependencies:

   .. code-block:: bash

      pip install -e <path_to_library>

Docker Installation
------------------
For containerized deployment:

.. code-block:: bash

    git clone https://github.com/IU-Capstone-Project-2025/total_control.git
    cd total_control
    docker compose up terminal --build
    docker compose exec terminal bash # in a new terminal

Hardware Setup
--------------
1. Connect your Cart-Pole system via USB
2. Verify device detection:

   .. code-block:: bash

      ls /dev/ttyUSB*  # Linux
      ls /dev/cu.*     # macOS

3. Grant permissions:

   .. code-block:: bash

      sudo chmod 666 <port> # Or use any other applicable way to set permissions 