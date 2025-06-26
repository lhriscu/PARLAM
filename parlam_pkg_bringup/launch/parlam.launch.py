from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    type_ns = LaunchConfiguration('type')
    type_ns_launch_arg = DeclareLaunchArgument(
        'type',
        default_value='static'
    )
    id_ns = LaunchConfiguration('id_experiment')
    id_ns_launch_arg = DeclareLaunchArgument(
        'id_experiment',
        default_value='1'
    )
    task_ns = LaunchConfiguration('task')
    task_ns_launch_arg = DeclareLaunchArgument(
        'task',
        default_value='handover'
    )
    language_ns = LaunchConfiguration('language')
    language_ns_launch_arg = DeclareLaunchArgument(
        'language',
        default_value='english'
    )
    return LaunchDescription([
       type_ns_launch_arg,
       task_ns_launch_arg,
       language_ns_launch_arg,
       id_ns_launch_arg ,
       Node(
           package="parlam_pkg",
           executable="speech_input_server",
           name="speech_input_server",
           parameters=[
#               {"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk-model-es-0.42'])},
               {"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk-model-en-us-0.22'])},
#               {"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk-model-small-ca-0.4'])},
               {"mic_id": "Plantronics Blackwire 3225 Series Analog Stereo"},
               {"num_inputs": 250},
               {"use_hw": True}
           ],
           remappings=[
               ('audio_data', '/operator/audio_in'),
               ('audio_info', '/operator/audio_info')
           ]
        ),
        Node(
            package="parlam_pkg",
            executable="speech_output_server",
            name="speech_output_server",
            parameters=[
                {"language_id": 'en'},                
                {"speaker_id": "Plantronics Blackwire 3225 Series Analog Stereo"}
            ]
        ),
        Node(
            package="parlam_pkg",
            executable="llm_server",
            name="llm_server"
        ),
        Node(
            package="parlam_pkg", 
            executable="parlam_client",
            name="parlam_client",
            parameters=[
                {"prompt_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'data'])},
                {"document_chromadb_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'data','info_eng.pdf'])},
                {"folder_chromadb_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'), 'chroma_database'])},
                {"greetings_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'data','greetings.json'])},
                {"language": language_ns},
                {"task": task_ns}, 
                {"type": type_ns}, 
                {"directory": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'conversations'])},
                {"save_conversation": True},
                {"id_experiment": id_ns}
            ],
            remappings=[
                ('/llm_action/_action/feedback', '/llm_action_bridge/llm_action/_action/feedback'),
                ('/llm_action/_action/status', '/llm_action_bridge/llm_action/_action/status'),
                ('/llm_action/_action/cancel_goal', '/llm_action_bridge/llm_action/_action/cancel_goal'),
                ('/llm_action/_action/get_result', '/llm_action_bridge/llm_action/_action/get_result'),
                ('/llm_action/_action/send_goal', '/llm_action_bridge/llm_action/_action/send_goal'),
            ]
        )
    ])

