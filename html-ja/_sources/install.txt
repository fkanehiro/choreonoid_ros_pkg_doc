=========
 Install
=========

This repository provides ROS support for Choreonoid

choreonoid\_ros\_pkg: Meta-package to build Choreonoid packages

choreonoid\_ros: Choreonoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic

jvrc\_models: Simulation models and tasks for JVRC

Before beginning the installation process, make sure you have installed wstool, catkin and rosdep:

.. code-block:: bash

   $ sudo apt-get install python-wstool python-catkin-tools python-rosdep
   $ sudo rosdep init
   $ rosdep update

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

Install dependent packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

To use the VRML based models in rviz, please install simtrans as well:

.. code-block:: bash

   $ sudo add-apt-repository ppa:hrg/daily
   $ sudo apt-get update
   $ sudo apt-get install python-pip openhrp meshlab imagemagick python-omniorb openrtm-aist-python python-numpy
   $ git clone https://github.com/fkanehiro/simtrans.git
   $ cd simtrans
   $ sudo pip install -r requirements.txt
   $ sudo python setup.py install

Build and install catkin packages:

.. code-block:: bash

   $ cd ~/catkin_ws
   $ catkin clean -b
   $ catkin config --install
   $ catkin build choreonoid_ros_pkg
   $ source install/setup.bash
