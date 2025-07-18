Electronics and Schematics
==========================

The Total Control system uses an ESP32-based controller board designed for real-time cart-pole control.

Connector Interfaces
--------------------
.. list-table:: 
   :header-rows: 1
   :widths: 20 30 50

   * - Label
     - Connector Type
     - Function
   * - **MOTOR**
     - HDR-F-2.54_1x6
     - Brushless motor control (CAN interface)
   * - **ANGLE**
     - HDR-F-2.54_1x5
     - Pole angle sensor (Infineon TLE5012B)
   * - **LINEAR**
     - HDR-F-2.54_1x4
     - Cart position encoder (SEAVDAN H9740)
   * - **END_BUT**
     - HDR-F-2.54_1x4
     - Rail end-stop switches
   * - **EMER**
     - HDR-F-2.54_1x5
     - Emergency stop button (hardware cutoff)
   * - **DATA**
     - 0.01×4
     - Serial debug (SWD interface)

Schematic Diagrams
------------------
.. figure:: ./_static/maf.png
   :alt: Scheme of electronics used in the Total Control project
   :align: center
   :width: 100%
   
   *Main circuit schematic (Rev 1.2) showing signal routing and component relationships*

PCB Layout
----------
.. figure:: ./_static/PCB_PCB_controller_3_2025-07-11.png
   :alt: Plate scheme
   :align: center
   :width: 100%
   
   *Final PCB design with optimized trace routing for noise immunity*
