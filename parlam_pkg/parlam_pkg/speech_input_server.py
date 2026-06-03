#!/usr/bin/env python3
# from parlam_interfaces.srv import Input
#from audio_common_msgs.msg import AudioData,AudioInfo
from parlam_interfaces.action import Input as input_action
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

import time
import vosk
import json
from collections import deque
import queue
import pygame
from pygame._sdl2 import AudioDevice
from pygame._sdl2 import AUDIO_S16
import pygame._sdl2.audio as sdl2_audio
import threading
import asyncio

vosk.SetLogLevel(-1)

class SpeechInputServer(Node):

    def __init__(self):
        """
        Initializes the SpeechInputServer node.
        """
        super().__init__('speech_input_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")

        # Declare default parameters
        if not self.has_parameter("silence_timeout"):
            self.get_logger().info("Declaring default value for silence_timeout parameter...")
            self.declare_parameter("silence_timeout", 1.2)
        if not self.has_parameter("listening_time"):
            self.get_logger().info("Declaring default value for listening_time parameter...")
            self.declare_parameter("listening_time", 10)
        if not self.has_parameter("debug"):
            self.get_logger().info("Declaring default value for debug parameter...")
            self.declare_parameter("debug", False)

        # self.declare_parameter("language_model", "/home/lhriscu/parlam_ws/install/parlam_pkg/share/parlam_pkg/models/vosk/vosk-model-en-us-0.22")
        # self.declare_parameter("mic_id", "Plantronics Blackwire 3225 Series Analog Stereo")
        # self.declare_parameter("use_hw", True)

        # Set parameters
        self.get_logger().info("Silence timeout: " + str(self.get_parameter("silence_timeout").value))
        self.get_logger().info("Max Listening time: " + str(self.get_parameter("listening_time").value))
        self.get_logger().info("Language model: " + self.get_parameter("language_model").value)
        self.get_logger().info("Mic id: " + self.get_parameter("mic_id").value)     
        #self.get_logger().info("Num partial results: " + str(self.get_parameter("num_inputs").value))
        self.get_logger().info("Debug: " + str(self.get_parameter("debug").value))    
        
        self.debug = self.get_parameter("debug").value

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        callback_group = ReentrantCallbackGroup()

        if(self.get_parameter("use_hw").value):
            pygame.init()
            self.audio=AudioDevice(
                devicename=self.get_parameter("mic_id").value,
                iscapture=True,
                frequency=44100,
                audioformat=AUDIO_S16,
                numchannels=1,
                chunksize=1024,
                allowed_changes=0,
                callback=self.callback)

            self.audio.pause(0)
            self.get_logger().info("Audio device created")
        else:
            self.audio_data_subs = self.create_subscription(
                AudioData,
                'audio_data',
                self.audio_data_callback,
                1,
                callback_group=callback_group)

        # Speech output action server
        self.input_action_server = ActionServer(
            self,
            input_action,
            'input_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
            ,callback_group=callback_group
        )
        
        #self.stop_event = threading.Event()
        self.q = queue.Queue()

        # self.partial_transcription=deque(maxlen=self.get_parameter("num_inputs").value)
        self.get_logger().info("Start loading Models")  
        self.rec = vosk.KaldiRecognizer(vosk.Model(self.get_parameter("language_model").value), 44100)

        self.get_logger().info("Speech Input Service is ready to receive inputs.")  
        
    def audio_data_callback(self, msg):
        channel_data=msg.data[1::2]
        self.get_logger().info("Received audio info")
        self.q.put(bytes(channel_data))

    # def audio_info_callback(self, msg):
    #     self.get_logger().info("Received audio info")

    def goal_callback(self, goal_request):
        '''
        Accepts or rejects a client request to begin an action.
        '''
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        '''
        Accepts or rejects a client request to cancel an action.
        '''
        self.get_logger().info('Received cancel request')
        #self.stop_event.set()
        return CancelResponse.ACCEPT

    def callback(self, audiodevice, audiomemoryview):
        """
        This method is called in the sound thread to receive audio stream conntinuously. 
        """
        self.q.put(bytes(audiomemoryview))

    def execute_callback(self, goal_handle):
        
        listen_time = goal_handle.request.listen_time
        if listen_time == 0:
            max_listen_time = self.get_parameter("listening_time").value
            self.get_logger().info("Received goal with time=0, using default listening_time")
        else:
            max_listen_time = listen_time
        result = input_action.Result()
        feedback = input_action.Feedback()

        def listen_once():

            self.q.queue.clear()
            self.rec.Reset()

            self.get_logger().info("Listening...")

            feedback.state = "Listening"
            goal_handle.publish_feedback(feedback)

            silence_timeout = self.get_parameter("silence_timeout").value
            filler_words = {"hm", "mh","and so", "ah", "oh", "the is the", "ah the", "oh the", "the is", "is the", "eh the", "uh the", "ha", "mmh", "sh", "uh", "um", "eh", "mmm", "the"}

            start_time = time.time()
            last_speech_time = time.time()

            accumulated_text = ""
            last_partial = ""

            while not goal_handle.is_cancel_requested:
                #self.get_logger().info("Found goal not canceled inside listen once...")
                # Safety timeout
                if (time.time() - start_time) > max_listen_time:
                    return accumulated_text if accumulated_text else "offconv"

                try:
                    data = self.q.get(block=True, timeout=0.1)

                    if self.rec.AcceptWaveform(data):
                        if self.debug:
                            self.get_logger().info("Waveform accepted as final by recognizer.")
                        result_stt = json.loads(self.rec.Result())
                        text = result_stt.get("text", "").strip()

                        if self.debug:
                            self.get_logger().info(f"Final STT result: '{text}'")

                        if text and text not in filler_words:
                            accumulated_text = text
                            last_speech_time = time.time()
                            if self.debug:
                                self.get_logger().info(
                                    f"Accumulated text updated to '{accumulated_text}' (final result)."
                                )
                            return accumulated_text

                    else:
                        partial_result = json.loads(self.rec.PartialResult())
                        partial = partial_result.get("partial", "").strip()
                        if self.debug and partial:
                            self.get_logger().info(f"Partial STT result: '{partial}'")

                        if partial and partial != last_partial:

                            # Ignore filler words
                            if partial.lower() not in filler_words:
                                if self.debug:
                                    self.get_logger().info(
                                        f"Accepted partial '{partial}' -> accumulated_text updated."
                                    )
                                accumulated_text = partial
                            last_speech_time = time.time()

                            last_partial = partial

                    # Finalize only after sustained silence
                    if accumulated_text:
                        silence_duration = time.time() - last_speech_time

                        if self.debug:
                            self.get_logger().debug(
                                f"Silence duration: {silence_duration:.2f}s (timeout={silence_timeout})"
                            )

                        if silence_duration > silence_timeout:
                            if self.debug:
                                self.get_logger().debug(
                                    f"Silence timeout reached. Returning text: '{accumulated_text}'"
                                )
                            return accumulated_text

                except Exception as e:
                    self.get_logger().error("Speech input error: " + str(e))
            if self.debug:
                self.get_logger().info("Found goal canceled inside listen once...")
            return None  # canceled
        
        while not goal_handle.is_cancel_requested:

            detected_text = listen_once()

            if detected_text is None:
                break  # canceled

            if detected_text != "offconv":
                if self.debug:
                    self.get_logger().info("Text detected, ending goal success...")
                goal_handle.succeed()
                result.success = True
                result.user_input = detected_text
                return result
            
            if detected_text == "offconv":
                if self.debug:
                    self.get_logger().info("No text detected and time limit reached, ending goal success...")
                goal_handle.succeed()
                result.success = True
                result.user_input = detected_text
                return result

            # If "offconv", just continue listening
            self.get_logger().info("No input detected, continuing listening...")
            feedback.state = "No input detected, continuing listening..."
            goal_handle.publish_feedback(feedback)

        # If canceled
        if self.debug:
            self.get_logger().info("Cancel goal from callback...")
        goal_handle.canceled()
        result.success = False
        result.user_input = "offconv"
        return result
      
def main(args=None):
    rclpy.init(args=args)

    speech_input_server = SpeechInputServer()
    executor = MultiThreadedExecutor()
    #executor= SingleThreadedExecutor()
    executor.add_node(speech_input_server)
    try:
        executor.spin()
    except Exception as e:
        speech_input_server.get_logger().error(f'An error occurred: {e}')
    finally:
        executor.shutdown()
        speech_input_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

