# Getting started

* In a terminal, run the command `cd` to ensure you are in your home directory. This isn't absolutely necessary, but will make it easier to find all your code in the future.
* Clone the repository `git clone git@gitlab-ex.sandia.gov:petricor/petricor_sim.git`. This repo already contains the high level directory.
* Once cloned, enter the workspace `cd petricor_sim`.
* Build the workspace `colcon build --symlink-install`.
    * It is possible the build doesn't work immediately. Usually, the errors will tell you exactly what you're missing and can be solved with a simple install. Usually along the lines of (don't run this) `sudo apt install ros-galactic-[whatever is missing]`.
* Source the build `source install/setup.bash`.
* Launch the simulation `ros2 launch platform_description spawn_platform_ros2.launch.py`.
* If this does not work and you receive an 'xacro' error, run this command: `sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control ros-galactic-xacro`. Then launch the simulation again.

# Tips for setting up your development environment
These tips aren't neccessary, but they may make your life a little easier.

* Add the following to your .bashrc
    * To open the .bashrc run `gedit ~/.bashrc`
* At the bottom of the file include the following:

```
source /opt/ros/galactic/setup.bash
source ~/petricor_sim/install/setup.bash
source /usr/share/gazebo/setup.sh

source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/galactic/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

```

# Arm control

* For the arm control, one of the three libraries will be used:
	1. [Robotics Toolbox Python](https://github.com/petercorke/robotics-toolbox-python)
	2. [MoveIt](https://moveit.ros.org/)
	3. [Openrave](http://wiki.ros.org/openrave/Tutorials/CreateKinematicReachability)

* At the time of writing, Robotics toolbox python is what is being used in arm_control.py

# Troubleshooting
* Ensure that Robotics Toolbox Python is up to date by running `pip install --upgrade roboticstoolbox-python`

# Running the platform simulation
* To launch the platform by itself, run `ros2 launch platform_description spawn_platform_ros2.launch.py`.
	* This should launch a Gazebo environment with the platform inside the world. 
	* At this time, the platform should be sitting still. Running the command `ros2 topic list` should show available topics.
* To run the platform controller, run `ros2 run platform_description pos_pub.py`.
	* The node has instructions once it is launched on how to control the vehicle. Use this to send the vehicle to waypoints or control it via `WASD`.

# Running the arm simulation
* To launch the arm sim, run `ros2 launch simple_arm spawn_simple_arm.launch.py`
	* This should launch an arm attached to the world frame. 
* To run the arm control, run `ros2 run simple_arm arm_control.py`
	* This controller takes an input of comma deliminated floats.
	* The right arm (as the left is uncontrolled at this moment) should snap to waypoints. For a smoother response, a trajectory generator is required (which is out of the scope of this project for the time being)

# Running the combined simulation
* To launch the simulation with the arm attached to the platform, run `ros2 launch platform_description combined_spawn.launch.py`.
	* Once launched, you should see the wheeled platform sitting on the ground plane and the arm mounted above it. 
* To launch the mobile platform control, run `ros2 run platform_description pos_pub.py`
* To launch the arm control, run `ros2 run simple_arm arm_control.py` seperatly. 
