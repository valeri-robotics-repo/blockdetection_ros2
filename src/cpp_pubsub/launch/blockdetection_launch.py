# Author: Addison Sears-Collins
# Date: August 27, 2021
# Description: Launch a basic mobile robot URDF file using Rviz.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
 
  # Create the launch description and populate
  ld = LaunchDescription()    
  
  blockdetection_node = Node(
        package="cpp_pubsub",
        executable="listener",
        parameters=[
            {"use_transform_system": True},
            {"source_frame": "camera_color_optical_frame2"},
            {"target_frame": "link_chassis2"},
            {"pointcloud_topic": "/camera/depth/color/points"},
            {"debug_level": "None"},  #None, Verbal, Visual
        ]
    )


  # Add any actions
  ld.add_action(blockdetection_node)

  return ld

