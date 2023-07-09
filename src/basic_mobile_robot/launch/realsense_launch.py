from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py']),
                    launch_arguments={'pointcloud.enable': 'true'}.items(),
                    #launch_arguments:{"pointcloud.enable":"true", "enable_gyro":"true"},
            )
    ])
