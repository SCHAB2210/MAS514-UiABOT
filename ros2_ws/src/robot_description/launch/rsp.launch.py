import os
from sys import executable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    # Define the package path at the start of the function
    pkg_path = os.path.join(get_package_share_directory('robot_description'))

    # Lidar configuration
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    # Rviz configuration
    rviz_config = os.path.join(pkg_path, 'config', 'uiabot_config.rviz')

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Define the BNO055 IMU nodes
    bno_node = Node(
        package='bno055',
        executable='bno_node.py',
        output='screen'
    )

    tf_bno_node = Node(
        package='bno055',
        executable='tf_bno.py',
        output='screen'
    )

    # Add the joint_state_publisher node
    node_joint_state_publisher = Node(
    package='raspi_sub',
    executable='joint_state_publisher.py',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
    )

    # Forward kinematic node
    kin_node = Node(
        package='raspi_sub',
        executable='forward_kinematic.py',
        output='screen'
    )

    # Kalman Filter for localization
    ekf_path = os.path.join(get_package_share_directory('robot_localization'))
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ekf_path + '/launch/ekf.launch.py'
        )
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
        ),

        # Lidar node
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            output='screen'
        ),

        # Launch the rest of the nodes
        node_robot_state_publisher,
        node_joint_state_publisher,
        kin_node,
        bno_node,
        tf_bno_node,
        ekf_launch
    ])

