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


Features comparison
===================

This section is describe a comparison of the feautures of the choreonoid_ros_pkg and the gazebo_ros_pkgs_.

The choreonoid_ros_pkg is implemented so as to be compatible with the gazebo_ros_pkgs, for that reason in principle it is to provide the same features.

Howerver, these may be differences depending on the differences of the each simulator. Also, not yet implemented features.

These are described below.

.. _gazebo_ros_pkgs: https://github.com/ros-simulation/gazebo_ros_pkgs

Gazebo Subscribed Topics
~~~~~~~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| topic     | /gazebo/set_link_state          | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| topic     | /gazebo/set_model_state         | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

Gazebo Published Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| parameter | /use_sim_time                   | /use_sim_time [1]_                             |
+-----------+---------------------------------+------------------------------------------------+
| topic     | /gazebo/parameter_descriptions  | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| topic     | /gazebo/parameter_updates       | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

Gazebo Published Topics
~~~~~~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| topic     | /clock                          | /clock                                         |
+-----------+---------------------------------+------------------------------------------------+
| topic     | /gazebo/link_states             | /choreonoid/link_states                        |
+-----------+---------------------------------+------------------------------------------------+
| topic     | /gazebo/model_states            | /choreonoid/model_states                       |
+-----------+---------------------------------+------------------------------------------------+
| topic     |                                 | /choreonoid/[world name]/physics/contacts [2]_ |
+-----------+---------------------------------+------------------------------------------------+

Gazebo Services
~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| service   | /gazebo/spawn_gazebo_model [3]_ | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/spawn_sdf_model         | /choreonoid/spawn_sdf_model [4]_               |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/spawn_urdf_model        | /choreonoid/spawn_urdf_model [4]_              |
+-----------+---------------------------------+------------------------------------------------+
| service   |                                 | /choreonoid/spawn_vrml_model [2]_ [4]_         |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/delete_model            | /choreonoid/delete_model                       |
+-----------+---------------------------------+------------------------------------------------+

State and properties getters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| service   | /gazebo/get_joint_properties    | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_link_properties     | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_link_state          | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_loggers             | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_model_properties    | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_model_state         | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_physics_properties  | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/get_world_properties    | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

State and properties setters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| service   | /gazebo/set_joint_properties    | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_link_properties     | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_link_state          | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_logger_level        | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_model_configuration | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_model_state         | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_parameters          | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/set_physics_properties  | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

Simulation control
~~~~~~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| service   | /gazebo/pause_physics           | /choreonoid/pause_physics [5]_                 |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/unpause_physics         | /choreonoid/unpause_physics [5]_               |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/reset_simulation        | /choreonoid/reset_simulation [5]_              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/reset_world             | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

Force control
~~~~~~~~~~~~~

+-----------+---------------------------------+------------------------------------------------+
| Category  | gazebo_ros_pkgs                 | choreonoid_ros_pkg                             |
+===========+=================================+================================================+
| service   | /gazebo/apply_body_wrench       | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/apply_joint_effort      | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/clear_joint_forces      | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+
| service   | /gazebo/clear_body_wrenches     | *not implemented*                              |
+-----------+---------------------------------+------------------------------------------------+

.. [1] A wall-clock time mode (use_sim_time=false) are currently not supported on the choreonoid_ros_pkg.
.. [2] This feature is unique of the choreonoid_ros_pkg.
.. [3] This feature is deprecated, alternative using to 'spawn_sdf_model'.
.. [4] Physical interference does not occur in models loaded from the limitation of Choreonoid specifications.
.. [5] This feature is inconsistent with the Choreonoid's GUI operation.

The each topics and services details described to from the next section.


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

This topic publish are made at a cycle (Hz) specified by the user.

.. image:: clock-property.png

The cycle default setting are 100.0 Hz.

/choreonoid/model\_states
~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of models are published to this topic.

This topic publish are made at a cycle (Hz) specified by the user.

.. image:: model-states-property.png

The cycle default setting are 100.0 Hz.

/choreonoid/links\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of links in models are published to this topic.

This topic publish are made at a cycle (Hz) specified by the user.

.. image:: link-states-property.png

The cycle default setting are 100.0 Hz.

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

/choreonoid/pause\_physics
~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes no argument. Pause the simulation.

/choreonoid/unpause\_physics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes no argument. Continue the paused simulation.

/choreonoid/spawn\_vrml\_model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Takes [model_name, model_data, namespace, pose, reference_frame] as arguments. Load the specified model to the simulation.

/choreonoid/delete\_model
~~~~~~~~~~~~~~~~~~~~~~~~~

Takes [model_name] as an argument. Delete the specified model from the simulation.

