from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import sys
# import argparse
import os
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import PathJoinSubstitution, TextSubstitution
def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('choirbot_examples'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # TURTLEBOT3_MODEL_i = LaunchConfiguration('TURTLEBOT3_MODEL', default='burger_cam')


    # TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    # TURTLEBOT3_MODEL = 'burger_cam'

    L=2
    seed=5
    for arg in sys.argv:
        if arg.startswith("L:="):
            L = int(arg.split(":=")[1])
        if arg.startswith("seed:="):
            seed = int(arg.split(":=")[1])

    # set rng seed
    np.random.seed(seed)

    # communication matrix
    N = 3

    Adj = np.array([
        [0, 1, 1],
        [1, 0, 1],
        [1, 1, 0]
    ])



    # generate matrix of desired inter-robot distances
    # adjacent robots have distance L
    # opposite robots have distance 2L
    W = np.array([
        [0, L, L],
        [L, 0, L],
        [L, L, 0]
    ])


    # generate coordinates of hexagon with center in the origin

    # P = np.array([
    #     [0,   0,                0],
    #     [L,   0,                0],
    #     [L/2, np.sqrt(3)/2 * L, 0]
    # ])

    P = np.array([
        [0,   0, 0],
        [L,   0, 0],
        [L/2, L, 0]
    ])


    # initial positions have a perturbation of at most L/3
    # P += np.random.uniform(-L/3, L/3, (N,3))
    
    print("Spawning Position of robots: ", P)

    # initialize launch description
    robot_launch = []       # launched after 10 sec (to let Gazebo open)
    launch_description = [] # launched immediately (will contain robot_launch)

    robot_models = ['burger_cam', 'burger_cam', 'waffle']
    # remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # included_launches = []
    # add executables for each robot
    for i in range(N):

        robot_name = robot_models[i]


        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        weights = W[i,:].tolist()
        position = P[i, :].tolist()

        # guidance
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_guidance', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i,
                'N': N,
                'in_neigh': in_neighbors,
                'out_neigh': out_neighbors,
                'weights': weights
            }]))
        
        # controller
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_controller', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{
                'agent_id': i
            }]))



        #robot state publisher
        # urdf_file_name = 'turtlebot3_' + robot_name + '.urdf'
        
        # urdf_path = os.path.join(get_package_share_directory('choirbot_examples'), 'urdf', urdf_file_name)

        # with open(urdf_path, 'r') as infp:
        #     robot_desc = infp.read()

        # robot_launch.append(Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     output='screen',
        #     namespace='agent_{}'.format(i),
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'robot_description': robot_desc,
        #         # 'robot_namespace': f'agent_{i}',
        #         'frame_prefix': f'agent_{i}/'
        #     }],
        #     remappings=remappings,
        #     # arguments=[urdf_path],
        # ))
        


        # robot_launch.append(Node(
        #     package="robot_state_publisher",
        #     namespace='agent_{}'.format(i),
        #     executable="robot_state_publisher",
        #     output="screen",
        #     parameters=[{
        #         "use_sim_time": use_sim_time, 
        #         "robot_description": robot_desc,
        #         "frame_prefix": f"agent_{i}/",
        #     }],
        #     # remappings=remappings,
        #     # arguments=[burger_urdf],
        # )
        # )
            
        
        
        # robot description
        # included_launch = (
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(
        #             os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        #         ),
        #         launch_arguments={
        #             'use_sim_time': use_sim_time,
        #             'robot_namespace': f'agent_{i}',
        #             'TURTLEBOT3_MODEL': robot_name,
        #             # 'new_background_r': TextSubstitution(text=str(colors['background_r']))
        #         }.items()
        # ))

        # launch_description.append(included_launch)


        # turtlebot spawner
        # launch_description.append(Node(
        #     package='choirbot_examples', executable='choirbot_turtlebot3_spawner', output='screen',
        #     parameters=[{
        #         'namespace': 'agent_{}'.format(i),
        #         'position': position,
        #         'TURTLEBOT3_MODEL': robot_name,
        #     }]
        # ))
    
    # include   er for gazebo
    # gazebo_launcher = os.path.join(launch_file_dir, 'gazebo_2.launch.py')


    # launch_description.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(gazebo_launcher)
    # ))    
    # # include delayed robot executables
    # timer_action = TimerAction(period=10.0, actions=[LaunchDescription(robot_launch)])
    # launch_description.append(timer_action)

    # return LaunchDescription(launch_description)
    return LaunchDescription(robot_launch)
