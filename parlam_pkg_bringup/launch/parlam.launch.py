from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    id_ns = LaunchConfiguration('id_experiment')
    id_ns_launch_arg = DeclareLaunchArgument(
        'id_experiment',
        default_value='1'
    )
    system_ns = LaunchConfiguration('system')
    system_ns_launch_arg = DeclareLaunchArgument(
        'system',
        default_value='1'
    )
    patient_ns = LaunchConfiguration('patient')
    patient_ns_launch_arg = DeclareLaunchArgument(
        'patient',
        default_value='1'
    )
    instructions_ns = LaunchConfiguration('instructions')
    instructions_ns_launch_arg = DeclareLaunchArgument(
        'instructions',
        default_value='False'
    )
    save_conversation_ns = LaunchConfiguration('save_conversation')
    save_conversation_ns_launch_arg = DeclareLaunchArgument(
        'save_conversation',
        default_value='False'
    )
    language_ns = LaunchConfiguration('language')
    language_ns_launch_arg = DeclareLaunchArgument(
        'language',
        default_value='english'
    )
    model_ns = LaunchConfiguration('model')
    model_ns_launch_arg = DeclareLaunchArgument(
        'model',
        default_value='qwen2.5:7B-instruct'
    )
    return LaunchDescription([
       language_ns_launch_arg,
       model_ns_launch_arg,
       id_ns_launch_arg ,
       system_ns_launch_arg,
       patient_ns_launch_arg,
       instructions_ns_launch_arg,
       save_conversation_ns_launch_arg,
       Node(
           package="parlam_pkg",
           executable="speech_input_server",
           name="speech_input_server",
           parameters=[
               #{"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk','vosk-model-es-0.42'])},
               {"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk','vosk-model-en-us-0.22'])},
#               {"language_model": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','vosk','vosk-model-small-ca-0.4'])},
               {"mic_id": "Plantronics Blackwire 3225 Series Analog Stereo"},
               {"num_inputs": 200},
                {"listening_time": 10},
                {"silence_timeout": 1.5},
               {"use_hw": True},
               {"debug": False}
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
                {"piper_config_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','piper', 'english.json'])},
                #{"piper_config_path": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'models','piper', 'spanish.json'])},
                {"speaker_id": "Plantronics Blackwire 3225 Series Analog Stereo"},
                {"debug": True}
            ]
        )
        ,
        Node(
            package="parlam_pkg",
            executable="llm_server",
            name="llm_server"
        ),
        Node(
            package="parlam_pkg", 
            executable="conversation_server",
            name="conversation_server",
            parameters=[
                {"model": model_ns},
                {"language": language_ns},
                {"directory": PathJoinSubstitution([FindPackageShare('parlam_pkg'),'conversations'])},
                {"save_conversation": save_conversation_ns},
                {"id_experiment": id_ns},
                {"debug": True},
                {"documents_path":  PathJoinSubstitution([FindPackageShare('parlam_pkg'),'data'])  },
                {"instructions": instructions_ns}
            ]
            ,
            remappings=[
                ('/conversation_action/_action/feedback', '/llm_action_bridge/conversation_action/_action/feedback'),
                ('/conversation_action/_action/status', '/llm_action_bridge/conversation_action/_action/status'),
                ('/conversation_action/_action/cancel_goal', '/llm_action_bridge/conversation_action/_action/cancel_goal'),
                ('/conversation_action/_action/get_result', '/llm_action_bridge/conversation_action/_action/get_result'),
                ('/conversation_action/_action/send_goal', '/llm_action_bridge/conversation_action/_action/send_goal'),
            ]
        )
    ])

