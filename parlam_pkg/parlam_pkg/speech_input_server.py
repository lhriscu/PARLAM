#!/usr/bin/env python3
from parlam_interfaces.srv import Input
# from audio_common_msgs.msg import AudioData,AudioInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import rclpy
from rclpy.node import Node

import vosk
import json
from collections import deque
import queue
import pygame
from pygame._sdl2 import AudioDevice
from pygame._sdl2 import AUDIO_S16
import pygame._sdl2.audio as sdl2_audio

vosk.SetLogLevel(-1)

class SpeechInputServer(Node):

    def __init__(self):
        """
        Initializes the SpeechInputServer node.
        """
        super().__init__('speech_input_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        # Set parameters
        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")
        self.get_logger().info("Language model: " + self.get_parameter("language_model").value)  
        self.get_logger().info("Mic id: " + str(self.get_parameter("mic_id").value))      
        self.get_logger().info("Num partial results: " + str(self.get_parameter("num_inputs").value))

        self.callback_group = ReentrantCallbackGroup()

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
                callback_group=self.callback_group)

            self.audio_info_subs = self.create_subscription(
                AudioInfo,
                'audio_info',
                self.audio_info_callback,
                1,
                callback_group=self.callback_group)

        self.srv = self.create_service(Input, 'speech_input', self.speech_input_callback,callback_group=self.callback_group)
                
        self.q = queue.Queue()
        self.partial_transcription=deque(maxlen=self.get_parameter("num_inputs").value)
        self.get_logger().info("Start loading Models")  
        self.rec = vosk.KaldiRecognizer(vosk.Model(self.get_parameter("language_model").value), 44100)

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        self.get_logger().info("Speech Input Service is ready to receive inputs.")  

    def audio_data_callback(self, msg):
        channel_data=msg.data[1::2]
        self.q.put(bytes(channel_data))

    def audio_info_callback(self, msg):
        self.get_logger().info("Received audio info")

    def publish_audio_content(self):
        """
        Runs the speech-to-text algorithm to get the transcription of the audio into text and returns it

        This function listens for audio input and publishes the transcribed text to the specified topic.
        """
        self.q.queue.clear()
        self.get_logger().info("Listening...")
        count=0
        while True:
            # Stop while when after X iterations (num_inputs value), there is no transcribed text.
            if (count == self.get_parameter("num_inputs").value) and (not self.partial_transcription):
                return "offconv"
            else:
                try:
                    data=self.q.get(block=True, timeout=0.1)
                    print(data)
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.Result())
                        if result["text"] in ("", "the"): #Background noise
                            return "offconv"
                        else: 
                            self.rec.Reset() 
                            self.get_logger().info("Result: " + str(result["text"]))
                            return str(result["text"])
                    else:
                        partial_result = json.loads(self.rec.PartialResult())
                        print(partial_result['partial'])
                        if partial_result['partial'] in ("", "the"):
                            count+=1
                            if count >= self.get_parameter("num_inputs").value: 
                                #In case that a noise was added to partial_transcription but then removed, and there are no further inputs
                                count = 0
                        else:
                            self.partial_transcription.append(partial_result['partial'])
                            self.get_logger().info(partial_result['partial'])
                            if (len(self.partial_transcription) == self.get_parameter("num_inputs").value) and (len(set(self.partial_transcription)) == 1):
                                self.partial_transcription.clear() 
                                self.rec.Reset()
                                return str(self.partial_transcription[0])

                except Exception as e:
                    self.get_logger().error("Error in speech input: " + str(e))

    def callback(self, audiodevice, audiomemoryview):
        """
        This method is called in the sound thread to receive audio stream conntinuously. 
        """
        self.q.put(bytes(audiomemoryview))

    def speech_input_callback(self, request, response):
        """
        Callback function for the 'speech_input' service.
        """
        response.user_input = self.publish_audio_content() 
        response.success = True
        response.message = ""
        return response

def main(args=None):
    rclpy.init(args=args)

    speech_input_server = SpeechInputServer()
    executor = MultiThreadedExecutor()
    executor.add_node(speech_input_server)
    executor.spin()

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

