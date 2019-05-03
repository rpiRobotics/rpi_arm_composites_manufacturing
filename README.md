Software Setup and Launch

The software for the ARM composites manufacturing assistant project uses ROS Kinetic on Ubuntu 16.04.4 Xenial LTS. wstool is used to assist with the configuration and maintenance of the working environment.  Follow these instructions to prepare and launch the software environment:
1.	Install Ubuntu and ROS if not already installed
a.	Ubuntu can be installed in a VirtualBox instance or as a direct install. Instructions are here:  http://releases.ubuntu.com/16.04/
b.	Install ros-kinetic-desktop-full : http://wiki.ros.org/kinetic/Installation/Ubuntu
2.	Configure git to cache https passwords (we will no longer use git+ssh for authentication)
a.	https://help.github.com/articles/caching-your-github-password-in-git/
3.	If your current catkin_ws environment has unsaved data, back up now!
4.	Initialize catkin_ws : http://wiki.ros.org/catkin/Tutorials/create_a_workspace
a.	source /opt/ros/kinetic/setup.bash
b.	mkdir -p ~/catkin_ws/src
c.	cd catkin_ws
d.	catkin_make_isolated (Be sure to use catkin_make_isolated!!!!)
e.	source devel_isolated/setup.bash (Be sure to use devel_isolated directory, not just devel directory)
5.	Use wstool to add all repos necessary to catkin_ws : http://wiki.ros.org/wstool
a.	wstool init src https://raw.githubusercontent.com/rpiRobotics/rpi_arm_composites_manufacturing/master/rpi_arm_composites_manufacturing.rosinstall
6.	Check status and update from wstool
a.	cd ~/catkin_ws
b.	wstool status -t src
c.	wstool update -t src
7.	Run rosdep to get other necessary packages
a.	rosdep install --from-paths src --ignore-src -r -y
8.	Patch pyassimp for MoveIt!
a.	Patch file is https://launchpadlibrarian.net/319496602/patchPyassim.txt
b.	File to be changed is /usr/lib/python2.7/dist-packages/pyassimp/core.py
9.	Install swig from source. The swig version that is part of Ubuntu is out of date.
a.	https://github.com/swig/swig
10.	Install Robot Studio on a Windows computer
a.	https://docs.google.com/document/d/1wvwB2GgZjKCHqpVx5xUWfZQm0sD64qzcKVFJn2B3254/edit?ts=5a272ae9
b.	Set the IP address for EGM to the Ubuntu ROS computer's IP address
11.	Install cygwin on the same computer, and make sure to install the "socat" package
12.	Run the following command on the Windows computer in cygwin
a.	socat TCP-LISTEN:60080,fork TCP:127.0.0.1:80
13.	Configure your local launch file on the ROS computer
a.	Local launch file contains settings for your specific machine
b.	Example contents for a Virtualbox computer: arm_composites_manufacturing_local.launch
14.	Launch the simulation environment
a.	roslaunch arm_composites_manufacturing_local.launch
