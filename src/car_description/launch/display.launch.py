import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('car_description'),
        'urdf',
        'car.urdf'
    )

    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    #joint_state_publisher_node = Node(
    #   package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher',
    #    parameters=[{
    #        'use_gui': False,
    #        'robot_description': urdf_content,
    #        'publish_default_positions': True   # Important: always publish /joint_states
    #    }]
    #)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_content
        }]
    )

    return LaunchDescription([
        #joint_state_publisher_node,
        robot_state_publisher_node
    ])
