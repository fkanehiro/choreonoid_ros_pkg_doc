=========
 Install
=========

This repository provides ROS support for Choreonoid

choreonoid\_ros\_pkg: Chorenoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic


To use the package, you first have to create catkin workspace:

.. code-block:: bash
   
   $ mkdir -p catkin_ws/src
   $ cd catkin_ws/src
   $ wstool init

Then, checkout choreonoid\_ros\_pkg under catkin\_ws/src folder:

.. code-block:: bash

   $ git clone https://github.com/fkanehiro/choreonoid_ros_pkg.git

Build catkin packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ catkin_make install
