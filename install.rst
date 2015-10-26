=========
 Install
=========

This repository provides ROS support for Choreonoid

choreonoid\_ros\_pkg: Chorenoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic


Before beginning the installation process, make sure you have installed wstool and catkin

.. code-block:: bash

   $ sudo apt-get install python-wstool python-catkin-tools

To use the package, you first have to create catkin workspace:

.. code-block:: bash
   
   $ mkdir -p ~/catkin_ws/src
   $ cd ~/catkin_ws/src
   $ catkin_init_workspace
   $ cd ~/catkin_ws
   $ catkin_make
   $ source devel/setup.bash

Then, checkout choreonoid\_ros\_pkg under catkin\_ws/src folder:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ wstool init
   $ wstool set choreonoid_ros_pkg https://github.com/fkanehiro/choreonoid_ros_pkg.git --git
   $ wstool update choreonoid_ros_pkg

Build catkin packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ catkin build choreonoid_ros
   $ catkin build choreonoid_plugins
