from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('alphasense'),
      'config',
      'param.yaml'
      )
    
    ns = LaunchConfiguration('namespace')

    return LaunchDescription([
        Node(
            package='alphasense',
            namespace=ns,
            executable='alphasense',
            parameters=[config],
        )
    ])

