=========
 Install
=========

About
=====

This plugin provides ROS support for Choreonoid.

This plugin are comprised of the following package.

- choreonoid\_ros\_pkg: Meta-package to build Choreonoid packages

- choreonoid\_ros: Choreonoid catkin package

- choreonoid\_plugin: Choreonoid plugins to publish ROS topic

- jvrc\_models: Simulation models and tasks for JVRC


Assumed environment for install
===============================

Verified that this plugin operates on the following platform.

- Ubuntu 14.04 64-bit PC (AMD) desktop + ROS Indigo

OS installation, please finish in advance.

How to install OS, please see http://www.ubuntu.com/download/desktop/install-ubuntu-desktop

.. note::

   If you use NVIDIA graphics card, must use of proprietary (not open source version) driver.

   If you use the open source version driver the Choreonoid with GLVisionSimulator is abnormally terminate.

ROS and required package installation is described in the next section.


Preliminary preparation for install
===================================

How to install ROS and required package.

Install ROS Indigo
------------------

Run the following command:

.. code-block:: bash

   $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
   $ sudo apt-get update
   $ sudo apt-get install ros-indigo-desktop-full
   $ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc

Procedure details, please see http://wiki.ros.org/indigo/Installation/Ubuntu

Install required package (without simtrans)
-------------------------------------------

Run the following command:

.. code-block:: bash

   $ sudo apt-get install python-rosinstall python-catkin-tools

After the intstallation, run the initialization and update of ROS package database:

.. code-block:: bash

   $ sudo rosdep init
   $ rosdep update

Run the following command, make sure without error:

.. code-block:: bash

   $ rosdep db
             :
           <snip>
             :
   $ echo $?
   0

Command rosdep details, please see http://wiki.ros.org/rosdep

Install required package (simtrans)
-----------------------------------

Run the following command:

.. code-block:: bash

   $ sudo add-apt-repository ppa:hrg/daily
   $ sudo apt-get update
   $ sudo apt-get install python-pip openhrp meshlab imagemagick python-omniorb openrtm-aist-python
   $ git clone https://github.com/fkanehiro/simtrans.git
   $ cd simtrans
   $ sudo pip install -r requirements.txt
   $ sudo python setup.py install

Confirm installation:

.. code-block:: bash

   $ which simtrans
   /usr/local/bin/simtrans
   $ simtrans -h
   usage: simtrans [-h] [-i FILE] [-o FILE] [-f FORMAT] [-c] [-b] [-t FORMAT]
                   [-p PREFIX] [-s] [-e SPGR] [-v]
   
   Convert robot simulation model from one another.
             :
           <snip>
             :
   $ echo $?
   0

Command simtrans details, please see https://github.com/fkanehiro/simtrans


Install Choreonoid ROS Plugin
=============================

How to install Choreonoid ROS plugin.

To use the package, you first have to create catkin workspace:

.. code-block:: bash
   
   $ mkdir -p ~/catkin_ws/src
   $ cd ~/catkin_ws
   $ catkin init

Then, checkout choreonoid\_ros\_pkg under catkin\_ws/src folder:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ wstool init
   $ wstool set choreonoid_ros_pkg https://github.com/fkanehiro/choreonoid_ros_pkg.git --git -y
   $ wstool update choreonoid_ros_pkg

Install dependent packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

Build and install catkin packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/indigo
   $ catkin config --install
   $ catkin build choreonoid_ros_pkg
   $ source install/setup.bash

To use the URDF/SDF based models in Choreonoid, please install sdfloader as well:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ wstool set choreonoid_sdfloader_plugin https://github.com/fkanehiro/choreonoid-sdfloader-plugin.git --git -y
   $ wstool update choreonoid_sdfloader_plugin
   $ cd ~/catkin_ws
   $ catkin build choreonoid_sdfloader_plugin

Changing ROS default setup.

Modify line of 'source /opt/ros/indigo/setup.bash' in the ~/.bashrc by any text editor:

.. code-block:: bash

   source ~/catkin_ws/install/setup.bash

- If reverting ROS default setup.

  Modify line of 'source ~/catkin_ws/install/setup.bash' in the ~/.bashrc by any text editor:

  .. code-block:: bash

     source /opt/ros/indigo/setup.bash

  After reverting, run the following command:

  .. code-block:: bash

     $ source ~/.bashrc


Update Choreonoid ROS Plugin
============================

How to update Choreonoid ROS plugin.

Run the following command:

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ wstool update choreonoid_ros_pkg
   $ wstool update choreonoid_sdfloader_plugin (*)
   $ cd ~/catkin_ws
   $ catkin clean -b -y
   $ catkin build choreonoid_ros_pkg
   $ catkin build choreonoid_sdfloader_plugin (*)
   $ source install/setup.bash

(*) If you have installed.


Setting of the CMake at the build of the Choreonoid
===================================================

If you want to including the other plugins at the time of build of the Choreonoid.
In the following steps, you can make it.

1. Create 'additional_cmake_args' file under the '~/catkin_ws/src/choreonoid_ros_pkg/choreonoid_ros' directory.

   .. code-block:: bash

      $ touch ~/catkin_ws/src/choreonoid_ros_pkg/choreonoid_ros/additional_cmake_args

2. Edit 'additional_cmake_args' file.

   .. code-block:: bash

      $ gedit ~/catkin_ws/src/choreonoid_ros_pkg/choreonoid_ros/additional_cmake_args

   e.g. If you want to append the OpenRTM plugin.

   .. code-block:: bash

      -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_OPENRTM_SAMPLES=ON

3. Run the following command.

   .. code-block:: bash

      $ cd ~/catkin_ws
      $ catkin clean -y
      $ catkin build choreonoid_ros_pkg
      $ catkin build choreonoid_sdfloader_plugin (*)
      $ source install/setup.bash

   (*) If you want to install.

For setting information, please refer to the http://choreonoid.org/en/manuals/1.5/install/options.html and the like.


Troubleshoot
============

Solve of a problem of after installation.

- If startup problem of 'roslaunch choreonoid_ros jvrc-1-rviz.launch'.

  Checking the catkin config value of 'Extending':

  .. code-block:: bash

     $ cd ~/catkin_ws
     $ catkin config
                :
              <snip>
                :
     Extending:             [env] /opt/ros/indigo
                :
              <snip>
                :

  - if case '[env | cached] <path to your home directory>/catkin_ws/devel:/opt/ros/indigo':

    Run the following command:

    .. code-block:: bash

       $ source install/setup.bash
       $ roslaunch choreonoid_ros jvrc-1-rviz.launch

  - if case '[env] /opt/ros/indigo':

    Run the following command:

    .. code-block:: bash

       $ export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/indigo
       $ source install/setup.bash
       $ roslaunch choreonoid_ros jvrc-1-rviz.launch

  - if case '[cached] /opt/ros/indigo' or other case:

    Run the following command:

    .. code-block:: bash

       $ catkin clean -y
       $ export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/indigo
       $ catkin build choreonoid_ros_pkg
       $ catkin build choreonoid_sdfloader_plugin (*)
       $ source install/setup.bash
       $ roslaunch choreonoid_ros jvrc-1-rviz.launch

    (*) If you want to install.

