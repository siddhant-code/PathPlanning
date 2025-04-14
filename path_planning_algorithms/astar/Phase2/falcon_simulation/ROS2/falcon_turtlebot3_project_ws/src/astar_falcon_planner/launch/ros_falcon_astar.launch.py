from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

static_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_pub',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'IMUSensor_BP_C_0']
)

def generate_launch_description():

    #Setup Launch Parameters
    start_position = LaunchConfiguration('start_position')
    end_position = LaunchConfiguration('end_position')
    robot_radius = LaunchConfiguration('robot_radius')
    clearance = LaunchConfiguration('clearance')
    delta_time = LaunchConfiguration('delta_time')
    goal_threshold = LaunchConfiguration('goal_threshold')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_distance = LaunchConfiguration('wheel_distance')
    rpms = LaunchConfiguration('rpms')

    start_position_launch_arg = DeclareLaunchArgument(
        'start_position',
        default_value='[0.0, 0.0, 0.0]'
    )
    end_position_launch_arg = DeclareLaunchArgument(
        'end_position',
        default_value='[0.0, 0.0, 0.0]'
    )
    robot_radius_launch_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.0'
    )
    clearance_launch_arg = DeclareLaunchArgument(
        'clearance',
        default_value='0.0'
    )
    delta_time_launch_arg = DeclareLaunchArgument(
        'delta_time',
        default_value='0.0'
    )
    goal_threshold_launch_arg = DeclareLaunchArgument(
        'goal_threshold',
        default_value='0.0'
    )
    wheel_radius_launch_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0'
    )
    wheel_distance_launch_arg = DeclareLaunchArgument(
        'wheel_distance',
        default_value='0.0'
    )
    rpms_launch_arg = DeclareLaunchArgument(
        'rpms',
        default_value='[50.0, 100.0]'
    )

    #Create Process to Launch Falcon
    launch_falcon_sim = ExecuteProcess(
        cmd=[[
            './Falcon.sh ',
            '-scenario=/home/ponaswin/AStarPlanningProject/Scenarios/AMRPathPlanning/AMRPathPlanning.usda'
            ]],
        cwd="/home/ponaswin/duality/falconsim-v5.1.0216/",
        shell=True
    )

    #Create Node to run AStar and Control TurtleBot
    astar_control_node = Node(
        package='astar_falcon_planner',
        # namespace='astar_ns',
        executable='falcon_amr_controller',
        name='falcon_amr_controller',
        parameters=[{
                'start_position': start_position,
                'end_position': end_position,
                'robot_radius': robot_radius,
                'clearance': clearance,
                'delta_time': delta_time,
                'goal_threshold': goal_threshold,
                'wheel_radius': wheel_radius,
                'wheel_distance': wheel_distance,
                'rpms': rpms,
            }]
    )

    return LaunchDescription([
        static_tf_node,
        start_position_launch_arg,
        end_position_launch_arg,
        robot_radius_launch_arg,
        clearance_launch_arg,
        delta_time_launch_arg,
        goal_threshold_launch_arg,
        wheel_radius_launch_arg,
        wheel_distance_launch_arg,
        rpms_launch_arg,
        astar_control_node,
        launch_falcon_sim,
    ])