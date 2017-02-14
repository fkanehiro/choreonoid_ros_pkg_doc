==============================
 Choreonoid ROS Plugin Manual
==============================

Install
=======

Please follow the procedure of :doc:`install`.


Run
===

Enter following commands to run ROS plugin enabled Choreonoid.

.. code-block:: bash
   
   $ roscore (on the different terminal)
   $ choreonoid

You have to configure AISTSimulator item to use Foward dynamics mode or High-gain dynamics mode, create and place WorldRos item under the World item, create and place BodyRos item under the robot you want to control.

Also, in the case of Foward dynamics mode a create BodyRosTorqueController item and in the case of High-gain dynamics mode a create BodyRosHighgainController item.

Please refer to :doc:`tutorial` on details.

.. note::

   The simulation time mode are currently supported.

   Howerver, the wall-clock time mode are currently not supported.

   Sorry, please use simulation time mode.

   The simulation time mode details, please see http://wiki.ros.org/Clock


ROS Topics
==========

Choreonoid ROS plugin provides following ROS topics, please refer to :doc:`tutorial` for actual use.

.. note::

   Data type of sensor\_msgs and geometry\_msgs provides by the BodyRos item.

   Data type of trajectory\_msgs provides by the BodyRosTorqueContorller or BodyRosHighgainController item.

   Other types are provides by the WorldRos item.

/[robotname]/joint\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each joint states are published to /[robotname]/joint\_states topic.

Data type of joint\_states topic is `sensor_msgs::JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_.


/[robotname]/[controlmode]/set\_joint\_trajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your control signal can be sent using /[robotname]/[controlmode]/set\_joint\_trajectory topic.

Data type of set\_joint\_trajectory topic is `trajectory_msgs::JointTrajectory <http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html>`_.


/[robotname]/[sensorname]
~~~~~~~~~~~~~~~~~~~~~~~~~

If there are sensors defined on the robot model, ROS plugin will generate topics correspond to each sensor.

Output from force sensors are published using data type `geometry_msgs::Wrench <http://docs.ros.org/api/geometry_msgs/html/msg/Wrench.html>`_.

Output from gyro sensors are published using data type `sensor_msgs::Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_.

Output from accel sensors are published using data type `geometry_msgs::Accel <http://docs.ros.org/api/geometry_msgs/html/msg/Accel.html>`_.

Output from range sensors are published using data type `sensor_msgs::LaserScan <http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html>`_.

Output from vision sensors are published using image transport.

For RGBD vision sensors, depth image is published in `sensor_msgs::PointCloud2 <http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html>`_ data type as well.

\/clock
~~~~~~~

Current simulation time is published to /clock topic.

/[worldname]/model\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of models are published to this topic.

/[worldname]/links\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of links in models are published to this topic.

/choreonoid/[worldname]/physics/contacts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Current contact state of links in models are published to this topic.

The output are published using data type `gazebo_msgs::ContactState <http://docs.ros.org/api/gazebo_msgs/html/msg/ContactState.html>`_.

It is decribed below summary each parameter of the topic.

* info:

  Simulation world name and time of contact (simulation time) will be output to this.

* collision1\_name:
* collision2\_name:

  The link name where contact occurred is output to this.
  This output format is '<body name>::<link name>::collision'.

* wrenches:

  It outputs force and torque generated at each contact position.
  This output is the value around center of mass in collision1\_name links.
  The relationship between wrench and contact position is linked with position of array.

* total\_wrench:

  It outputs sum of forces and torques

* contact\_positions:

  It outputs the contact position in world coordinates.

* contact\_normals:

  It outputs the contact noraml.

  .. image:: contacts-state-normal.png

  The relationship between contact normal and contact position is linked with position of array.

* depths:

  It outputs penetration depth.
  The relationship between depth and contact position is linked with position of array.

This topic publish are made at a cycle (Hz) specified by the user.

.. image:: contacts-state-property.png

The cycle default setting are 100.0 Hz.


ROS Services
============

Following ROS services are provided to control the simulation.

.. note::

   All services are provides by the WorldRos item.

/[worldname]/pause\_physics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes no argument. Pause the simulation.

/[worldname]/unpause\_physics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes no argument. Continue the paused simulation.

/[worldname]/spawn\_vrml\_model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes [model_name, model_data, namespace, pose, reference_frame] as arguments. Load the specified model to the simulation.

/[worldname]/delete\_model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes [model_name] as an argument. Delete the specified model from the simulation.

