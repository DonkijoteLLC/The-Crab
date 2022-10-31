#!/usr/bin/env python3

# spawn_simple_arm.launch.py

# Written by: Isaac Seslar

# Description:
# 	This script utilizes the ROS 2 launch package to start a simulation
# 	of the Petricor platform within a Gazebo environment.

import launch 
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	# urdf_file_name  = 'urdf/dual_scara_arm.urdf'

	use_sim_time = LaunchConfiguration('use_sim_time', default='true')

	platform_description_path = os.path.join(
		get_package_share_directory('simple_arm')
		)

	# platform_gaz_plugin_path = platform_description_path + '/../../lib/platform_description'

	gazebo = IncludeLaunchDescription(
				PythonLaunchDescriptionSource([os.path.join(
					get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
				launch_arguments = \
            	{'world'   : platform_description_path + '/worlds/petricor_basic.world'}.items()
				)
	xacro_file = os.path.join(platform_description_path,
								'urdf',
								'dual_scara_arm.xacro')

	doc = xacro.parse(open(xacro_file))
	xacro.process_doc(doc)
	params = {'robot_description': doc.toxml()}


	spawn_entity = Node(package='gazebo_ros', 
						executable='spawn_entity.py',
						arguments=['-topic', 'robot_description',
								   '-entity', 'simple_arm'],
						output='screen')

	node_robot_state_pub = Node(
		package    = 'robot_state_publisher',
		executable = 'robot_state_publisher',
		output     = 'screen',
		parameters = [params],
		arguments  = [xacro_file])

	# Start joint_state_broadcaster
	load_joint_state_controller = ExecuteProcess(
		cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
			 'joint_state_broadcaster'],
		output='screen'
	)

	load_joint_position_controller = ExecuteProcess(
		cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'position_controllers'],
		output='screen'
	)

	# Set joint_state_broadcaster to start after spawn_entity action complete
	spawn_event = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=spawn_entity,
				on_exit=[load_joint_state_controller],
			))

	# Set effort_controllers to start after joint_state_broadcaster has  started
	joint_event = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=load_joint_state_controller,
				on_exit=[load_joint_position_controller],
			))

	ld.add_action(spawn_event)
	ld.add_action(joint_event)
	ld.add_action(gazebo)
	ld.add_action(node_robot_state_pub)
	ld.add_action(spawn_entity)

	return ld
