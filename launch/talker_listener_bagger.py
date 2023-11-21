import launch
import launch_ros
import time
def generate_launch_description():
    # Create parameters for the nodes
    talker_f = launch.substitutions.LaunchConfiguration('talker_f')
    record_enabled = launch.substitutions.LaunchConfiguration('record_enabled')
    replay_only = launch.substitutions.LaunchConfiguration('replay_only')
    bag_file = launch.substitutions.LaunchConfiguration('bag_file')
    # Create the launch arguments
    bag_file_arg = launch.actions.DeclareLaunchArgument('bag_file', default_value='recorder'+str(time.time())+'.bag')
    replay_only_arg = launch.actions.DeclareLaunchArgument('replay_only', default_value='False')
    record_enabled_arg = launch.actions.DeclareLaunchArgument('record_enabled', default_value='True')
    talker_freq = launch.actions.DeclareLaunchArgument('talker_f', default_value='10.0')
    # Create the nodes
    talker = launch_ros.actions.Node(
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression(['not ', replay_only])),
        package = 'cpp_pubsub',
        executable = 'talker',
        remappings = [("custom_topic", "chatter")],
        name='talker',
        output = 'screen',
        parameters = [{'talker_f': talker_f}]
        )
    listener = launch_ros.actions.Node(
        # condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression([replay_only])),
        package = 'cpp_pubsub',
        executable = 'listener',
        name="listener",
        remappings = [("custom_topic", "chatter")],
        output = 'screen'
        )
    
    # Create an executable that will run the bag recorder for 
    recorder = launch.actions.ExecuteProcess(
        # Conditionally run the recorder based on the launch argument
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression([record_enabled, ' and ', 'not ', replay_only])),
        cmd = ['ros2', 'bag', 'record', '-a' , '-o', bag_file, '-d', '15'],
        output = 'screen')
    
    bag_player = launch.actions.ExecuteProcess(
        # Conditionally run the bag player based on the launch argument
        condition = launch.conditions.IfCondition(launch.substitutions.PythonExpression([replay_only])),
        cmd = ['ros2', 'bag', 'play', bag_file],
        output = 'screen')
    
    return launch.LaunchDescription([
        # Add the launch arguments
        bag_file_arg,
        replay_only_arg,
        record_enabled_arg,
        talker_freq,
        # Add the nodes to the launch description
        talker,
        listener,
        # Add the recorder to the launch description
        recorder,
        bag_player,
        ])