from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 👇 Declare the LaunchConfiguration objects
namespace = LaunchConfiguration('namespace')
distance = LaunchConfiguration('distance')

# 👇 Declare the arguments to expose them externally
namespace_arg = DeclareLaunchArgument(
    'namespace',
    default_value='agent_0',
    description='Namespace for the robot'
)

distance_arg = DeclareLaunchArgument(
    'distance',
    default_value='1.0',
    description='Distance to move forward (in meters)'
)

distance_set_node = Node(
    package='choirbot_examples',
    executable='robot_mover',
    namespace=namespace,
    name='robot_mover_node',
    parameters=[{
        'distance': distance
    }]
)

def generate_launch_description():
    return LaunchDescription([
        namespace_arg,
        distance_arg,
        distance_set_node
    ])
