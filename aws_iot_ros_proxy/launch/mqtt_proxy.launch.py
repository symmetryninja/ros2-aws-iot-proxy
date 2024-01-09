import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from std_msgs.msg import Empty
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    launch_description = LaunchDescription()

    # use an environment variable for the path, default to ~/iot_params.json
    try:
        path_to_config = EnvironmentVariable('AWS_IOT_PARAMETER_FILE')
    except:
        path_to_config = "~/iot_params.json"

    ## MQTT <> ROS bridge
    launch_description.add_action(
        Node(
            package='aws_iot_ros_proxy',
            executable='aws_iot_ros_proxy',
            name='mqtt_proxy',
            parameters= [
                {'path_for_config': path_to_config }
            ]
        ))

    return launch_description
