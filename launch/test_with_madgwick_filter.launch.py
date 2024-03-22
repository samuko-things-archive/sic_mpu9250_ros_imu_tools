import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('sic_mpu9250'))
    imu_filter_config_file = os.path.join(pkg_path,'config','imu_filter_test_params.yaml')
    sic_mpu9250_config_file = os.path.join(pkg_path,'config','sic_mpu9250_params.yaml')
    
    imu_tools_madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[imu_filter_config_file],
    )

    sic_mpu9250_node = Node(
        package='sic_mpu9250',
        executable='sic_mpu9250',
        name='sic_mpu9250',
        output='screen',
        parameters=[sic_mpu9250_config_file],
    )


     # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(imu_tools_madgwick_filter_node)
    ld.add_action(sic_mpu9250_node)
    
    return ld      # return (i.e send) the launch description for excecution
    
