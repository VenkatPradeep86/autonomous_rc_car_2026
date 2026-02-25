from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory('car_description'),
        'urdf',
        'car.urdf'   # <<< CHANGE THIS NAME
    )

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

    ])
