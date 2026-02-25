from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def include_launch(pkg, rel_path, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), rel_path)
        ),
        launch_arguments=launch_arguments.items() if launch_arguments else None
    )

def generate_launch_description():
    
    # --- 1. SENSORS ---
    
    # RPLIDAR (The Main Eye)
    rplidar = include_launch(
        'rplidar_ros',
        'launch/rplidar.launch.py',
        launch_arguments={}
    )

    # ULTRASONIC (The Close-Range Safety) - RE-ENABLED
    ultrasonic = Node(
         package='ultrasonic_sensor',           # Confirmed Package Name
         executable='ultrasonic_node.py',       # Confirmed Script Name
         name='ultrasonic_node',
         output='screen',
         respawn=True
    )

    # WHEEL ENCODERS (The Odometer)
    wheel_encoder = Node(
        package='wheel_encoder',
        executable='wheel_encoder_node.py',
        name='wheel_encoder_node',
        output='screen',
        respawn=True
    )

    # --- 2. ACTUATORS ---
    
    # MOTOR CONTROL
    drive_control = Node(
        package='drive_control',
        executable='drive_node.py',
        name='drive_node',
        output='screen',
        respawn=True
    )

    # --- 3. STATE ESTIMATION & TF ---

    # ROBOT STATE PUBLISHER (URDF) - HEADLESS MODE
    # Replaced 'display' with 'bringup' to avoid opening RViz on the car
    bringup = include_launch(
        'car_bringup',
        'launch/bringup.launch.py',  
    )

    # ODOMETRY PUBLISHER
    odom = Node(
        package='odometry_publisher',
        executable='car_odometry.py',
        name='car_odometry',
        output='screen',
        respawn=True
    )

    # --- Group hardware nodes (Optional grouping) ---
    hw_group = GroupAction([
        drive_control 
    ])

    return LaunchDescription([
        bringup,        # Publishes TF / Robot Model
        rplidar,        # Lidar Data
        ultrasonic,     # Ultrasonic Data
        wheel_encoder,  # Encoder Data
        odom,           # Odometry Calculation
        hw_group,       # Motor Control
    ])
