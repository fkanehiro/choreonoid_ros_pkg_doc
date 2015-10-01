================================
 Choreonoid ROS Plugin Tutorial
================================

Run preconfigured project
=========================

You can use preconfigured project prepared for this tutorial.

.. code-block:: bash
   
   $ ./devel/bin/choreonoid src/choreonoid_ros_pkg/choreonoid_plugins/test/jvrc1test.cnoid --start-simulation

Configure the project by hand
=============================

Or you can prepare project of your own.

To prepare your own project:

1. Create World item and create Body item by opening the robot model data.
2. (Optional) Create and place ViewSimulator item under the robot you want to get image input.
3. Create and place BodyRos item under the robot you want to control.
4. Configure AISTSimulator item to use High-gain dynamics mode.

.. image:: highgain.png

5. Create and place WorldRos item under the world you want to control.

Item view should be structured as follows after the above configuration.

.. image:: itemview.png

Finally, click "start simulation" button to enable the ROS functions.


Use ROS utility commands to monitor the message
===============================================

List available topics:

.. code-block:: bash
   
   $ rostopic list

Print input of force sensor:
   
.. code-block:: bash

   $ rostopic echo /JVRC_1/lfsensor

Display camera input:

.. code-block:: bash
   
   $ rosrun image_view image_view image:=/JVRC_1/rcamera

Use Python script to send command to the robot
==============================================

Following example sends each trajectory commands to joint J1 and J2 of PA10 robot.

.. literalinclude:: test-jointtrajectory-pa10.py
   :language: python

Use ROS utility commands to control the simulation
==================================================

Pause the simulation:

.. code-block:: bash

   $ rosservice call /AISTSimulator/pause_physics

Continue the paused simulation:
   
.. code-block:: bash

   $ rosservice call /AISTSimulator/unpause_physics

Use Python script to spawn the model
====================================

Following example loads box model dynamically to current simulation.

.. literalinclude:: test-model-spawn.py
   :language: python

