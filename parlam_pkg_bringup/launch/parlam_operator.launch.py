from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    language_ns = LaunchConfiguration('language_id')
    language_ns_launch_arg = DeclareLaunchArgument(
        'language_id',
        default_value='en'
    )
    return LaunchDescription([
        language_ns_launch_arg,
        Node(
            package="parlam_pkg",
            executable="speech_output_server",
            name="speech_output_server",
            parameters=[
                {"language_id": language_ns},
                {"speaker_id": "Plantronics Blackwire 3225 Series Analog Stereo"}
            ]
        ),
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
    ])

