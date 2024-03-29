from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    subscribe_lidar_node = Node(
        package = "subscribe_lidar",
        executable = "subscribe_lidar_node",
    )

    findGap_node = Node(
        package = "findgap_uart",
        executable = "findgap_uart_node",
    )

    ldlidar_node = Node(
      package='ldlidar_stl_ros2',
      executable='ldlidar_stl_ros2_node',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': False},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 180.0},
        {'angle_crop_max': 360.0}
      ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld06',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
    )    


    ld.add_action(subscribe_lidar_node)
    ld.add_action(findGap_node)
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)
    
    return ld