#!/usr/bin/env python3
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from parlam_interfaces.srv import Output

import rclpy
from rclpy.node import Node

import pygame 
from gtts import gTTS
from io import BytesIO
from pydub.playback import play

class SpeechOutputServer(Node):

    def __init__(self):
        """
        Initializes the SpeechOutputServer node.
        """
        super().__init__('speech_output_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        # Set parameters
        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")
        self.get_logger().info("Language id: " + self.get_parameter("language_id").value)  
        self.get_logger().info("Speaker id: " + self.get_parameter("speaker_id").value)           

        self.srv = self.create_service(Output, 'speech_output', self.speech_output_callback)
                
        if pygame.mixer.get_init():
            pygame.mixer.quit()
        pygame.mixer.init(devicename=self.get_parameter("speaker_id").value)
        
        self.get_logger().info("\n --------- \n LOGS \n ---------")
        self.get_logger().info("Speech Output Service is ready to receive inputs.")  


    def synthesize_speech(self, text):
        """
        Generates speech from text using gTTS.
        
        Args:
            text (String): Message to be passed to TTS.
        """
        fp = BytesIO()
        try:
            tts = gTTS(text=text, lang=self.get_parameter("language_id").value, slow=False)
            tts.write_to_fp(fp)
            fp.seek(0)
            return fp
        except Exception as e:
            self.get_logger().error(f"gTTS synthesis failed: {e}")
            return fp

    def play_audio(self, fp):
        """
        Plays the audio through the selected device.

        Args:
            fp (Bytes): Audio data.
        """
        try:
            pygame.mixer.music.load(fp)
        except pygame.error as e:
            self.get_logger().error(f"pygame error: failed to load audio: {e}")
            return
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pass    

    def speech_output_callback(self, request, response):
        """
        Callback function for the 'speech_output' service.
        """
        fp = self.synthesize_speech(request.answer)
        self.play_audio(fp)
        response = Output.Response()
        response.success = True
        response.message = ""
        return response

def main(args=None):
    rclpy.init(args=args)

    speech_output_server = SpeechOutputServer()

    rclpy.spin(speech_output_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

