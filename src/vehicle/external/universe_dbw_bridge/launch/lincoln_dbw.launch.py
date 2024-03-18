from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def load_parameters_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        parameters = yaml.safe_load(file)
    return parameters

def generate_launch_description():
    package_name = 'universe_dbw_bridge'
    executable_name = 'Lincoln_MKZ_Bridge'

    launch_file_dir = os.path.dirname(__file__)

    params_file_path = os.path.join(launch_file_dir, '..', 'config', 'vehicle_params.yaml')

    parameters = load_parameters_from_yaml(params_file_path)

    return LaunchDescription([
        Node(
            package=package_name,
            executable=executable_name,
            name=executable_name,
            output='screen',
            parameters=[parameters]
        )
    ])
