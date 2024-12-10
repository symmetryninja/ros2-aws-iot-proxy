import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo
from std_msgs.msg import Empty
from launch.substitutions import EnvironmentVariable
import os

def generate_launch_description():
    launch_description = LaunchDescription()

    # use an environment variable for the path, default to ~/iot_params.json
    try:
        path_to_config = os.environ['AWS_IOT_PARAMETER_FILE']
        if not path_to_config or path_to_config == "":
            path_to_config = "~/certs/aws_iot_config.json"
    except:
        path_to_config = "~/certs/aws_iot_config.json"

    # use an environment variable for the iot endpoint
    try:
        iot_endpoint = os.environ['AWS_IOT_ENDPOINT']
        if not iot_endpoint or iot_endpoint == "":
            iot_endpoint = ""
    except:
        iot_endpoint = ""

    ## MQTT <> ROS bridge
    launch_description.add_action(
        Node(
            package='aws_iot_ros_proxy',
            executable='aws_iot_ros_proxy',
            name='mqtt_proxy',
            parameters= [
                {'path_for_config': path_to_config },
                {'iot_endpoint': iot_endpoint },
            ]
        ))

    return launch_description
