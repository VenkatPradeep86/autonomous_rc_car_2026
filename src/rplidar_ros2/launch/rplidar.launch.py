from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                # Use the most common port, but be ready to change to /dev/ttyUSB1
                'serial_port': '/dev/ttyUSB0', 
                'serial_baudrate': 115200,  
                # Ensure this matches the link name in your car.urdf
                'frame_id': 'laser', 
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
