import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    args = sys.argv[1:]

    sdf_robobo = os.path.join(get_package_share_directory('robobo_gazebo'), 'models/robobo/model.sdf')
    assert os.path.exists(sdf_robobo)

    install_dir = get_package_prefix('robobo_gazebo')

    model_dir = os.path.join(get_package_share_directory('robobo_gazebo'), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + model_dir
    else:
        os.environ['GAZEBO_MODEL_PATH'] = model_dir

    lib_dir = os.path.join(install_dir, 'lib')

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + lib_dir
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(install_dir, '/lib')

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if (key.isupper()):
                envs[key] = os.environ[key]
    except Exception as e:
        print("Error with Envs: " + str(e))
        return None

    robot_name = 'robobo'
    robot_namespace = 'robobo'
    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen',
            env=envs
        ),
        Node(package='robobo_gazebo', node_executable='spawn_robobo.py', 
            arguments=[robot_name, robot_namespace, '0.0', '0.0', '0.1'], output='screen')
    ])
    return ld