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

You have to configure AISTSimulator item to use Foward dynamics mode or High-gain dynamics mode.
Create and place BodyRos item under the robot you want to control.

Also, in the case of Foward dynamics mode a create BodyRosTorqueController item and in the case of High-gain dynamics mode a create BodyRosHighgainController item.

Please refer to :doc:`tutorial` on details.


ROS Topics
==========

Choreonoid ROS plugin provides following ROS topics, please refer to :doc:`tutorial` for actual use.

/[robotname]/[controlmode]/joint\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each joint states are published to /[robotname]/[controlmode]/joint\_states topic.

Data type of joint\_states topic is `sensor_msgs::JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_.


/[robotname]/[controlmode]/set\_joint\_trajectory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Your control signal can be sent using /[robotname]/[controlmode]/set\_joint\_trajectory topic.

Data type of set\_joint\_trajectory topic is `trajectory_msgs::JointTrajectory <http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html>`_.


/[robotname]/[sensorname]
~~~~~~~~~~~~~~~~~~~~~~~~~

If there are sensors defined on the robot model, ROS plugin will generate topics correspond to each sensor inputs.

Data type used for force sensors are `geometry_msgs::Wrench <http://docs.ros.org/api/geometry_msgs/html/msg/Wrench.html>`_.

Data type used for rate gyro sensors are `sensor_msgs::Imu <http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html>`_.

Data type used for accel sensors are `geometry_msgs::Accel <http://docs.ros.org/api/geometry_msgs/html/msg/Accel.html>`_.

Data type used for range sensors are `sensor_msgs::LaserScan <http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html>`_.

Input for vision sensors are published using image transport.

For RGBD vision sensors, depth image is published in `sensor_msgs::PointCloud2 <http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html>`_ data type as well.

\/clock
~~~~~~~

Current simulation time is published to /clock topic.

/[worldname]/model\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of models are published to this topic.

/[worldname]/links\_states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Current position and attitude of links in models are published to this topic.

ROS Services
============

Following ROS services are provided to control the simulation.

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

