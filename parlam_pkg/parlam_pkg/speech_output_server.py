#!/usr/bin/env python3
import os

from parlam_interfaces.action import Output as output_action
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

import subprocess
import threading
import queue
#import sounddevice as sd

import numpy as np
import json
import urllib.request
import pygame
import pygame._sdl2.audio as sdl2_audio
import time

# Stream with Pygame and Piper TTS
class SpeechOutputServer(Node):

    def __init__(self):
        """
        Initializes the SpeechOutputServer node.
        """
        super().__init__('speech_output_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        # self.declare_parameter("speaker_id", "Plantronics Blackwire 3225 Series Analog Stereo")
        # self.declare_parameter("piper_config_path", "/home/lhriscu/parlam_ws/install/parlam_pkg/share/parlam_pkg/models/piper/english.json")
        # Set parameters
        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")

        if not self.has_parameter("debug"):
            self.get_logger().info("Declaring default value for debug parameter...")
            self.declare_parameter("debug", False)
            
        #self.get_logger().info("Language id: " + self.get_parameter("language_id").value)  
        self.get_logger().info("Speaker id: " + self.get_parameter("speaker_id").value)  
        self.get_logger().info("Piper config path: " + self.get_parameter("piper_config_path").value)  


        self.debug = self.get_parameter("debug").value

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        # Init pygame
        if pygame.mixer.get_init():
            pygame.mixer.quit()

        pygame.mixer.init(frequency=22050,
                        size=-16,
                        channels=1,
                        devicename=self.get_parameter("speaker_id").value)

        # TTS Voice Start
        with open(self.get_parameter("piper_config_path").value, "r") as f:
            piper_config= json.load(f)
        self.voice_path = self.ensure_voice_from_config(
            piper_config,
            piper_config["voice_name"],
            self.get_logger()
        )

        # Start piper
        self.piper = subprocess.Popen(
            ["piper", "--model", self.voice_path, "--output_raw"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # Event to stop the audio thread
        self.stop_audio = threading.Event() 

        # Callback groups
        action_cb_group = ReentrantCallbackGroup()
        sub_cb_group = ReentrantCallbackGroup()

        # Speech output action server
        self.output_action_server = ActionServer(
            self,
            output_action,
            'output_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=action_cb_group
        )

        # Input text subscriber
        self.text_subscriber = self.create_subscription(
            String,
            '/output_text',
            self.text_callback,
            10,
            callback_group=sub_cb_group
        )

        # Lock condition tests and queues
        self.text_queue = queue.Queue()
        self.audio_queue = queue.Queue()
        self.stop_audio = threading.Event()
        self.state_lock = threading.Lock()
        self.llm_done_event = threading.Event()

        # Bools to monitor audio generation and playback
        self.is_playing = False
        self.is_generating = False

        self.it=0

        self.get_logger().info("Speech Output Service is ready to receive inputs.")        

    def ensure_voice_from_config(self, config, voice_name, logger: None):
        base_dir = os.path.expanduser(config["base_dir"])
        voice = config["voices"][voice_name]

        os.makedirs(base_dir, exist_ok=True)

        onnx_path = os.path.join(base_dir, voice["files"]["onnx"])
        json_path = os.path.join(base_dir, voice["files"]["json"])

        if os.path.exists(onnx_path) and os.path.exists(json_path):
            logger.info(f"Piper voice ready: {voice_name}")
            return onnx_path

        logger.info(f"Downloading Piper voice: {voice_name}")

        try:
            urllib.request.urlretrieve(voice["urls"]["onnx"], onnx_path)
            urllib.request.urlretrieve(voice["urls"]["json"], json_path)

        except Exception as e:
            logger.error(f"Failed to download Piper voice: {voice_name}, Error: {e}")
            return None

        return onnx_path
    
    def audio_generator(self, stop_audio):
        """
        Background thread: picks text from the queue and generates Piper bytes file for TTS.
        """
        # if self.debug:
        #     self.get_logger().info(f"Thread ID Audio generator: {threading.get_ident()}")
        while not stop_audio.is_set():
            try:
                text = self.text_queue.get(timeout=0.01)  # wait for new text
                if self.debug:
                    self.get_logger().info("Processing text from queue: " + text)
                self.is_generating = True
                piper = subprocess.Popen(
                        ["piper", "--model", self.voice_path, "--output_raw"],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE
                    )
                audio, err = piper.communicate(text.encode())

                if err:
                    self.get_logger().info("Piper error:", err.decode())

                if audio:
                    if not stop_audio.is_set():
                        if self.debug:
                            self.get_logger().info("Piper: put audio to queue: " + text)
                        self.audio_queue.put(audio)
                # else:
                #     self.get_logger().info("Piper: no audio generated")
            except queue.Empty:
                continue
            finally: 
                self.is_generating = False


    def playback_audio(self):
        # if self.debug:
        #     self.get_logger().info(f"Thread ID: {threading.get_ident()}")
        while not self.stop_audio.is_set():
            try:
                audio = self.audio_queue.get(timeout=0.05)
                self.is_playing = True
                if self.debug:
                    self.get_logger().info("Processing audio...")
            except queue.Empty:
                # if self.debug:
                #     self.get_logger().info("Empty audio queue")
                continue
            try:
                
                sound = pygame.mixer.Sound(buffer=audio)
                channel = sound.play()
                self.is_playing = True

                # while pygame.mixer.get_busy() and not self.stop_audio.is_set():
                #     time.sleep(0.001)

                while channel.get_busy():
                    if self.stop_audio.is_set():
                        channel.stop()
                        break
                    time.sleep(0.001)

            except pygame.error as e:
                self.get_logger().error(f"Playback error: {e}")
            finally:
                self.is_playing = False

    def flush_queue(self):
        with self.state_lock:
            # Flush text queue
            while True:
                try:
                    self.text_queue.get_nowait()
                except queue.Empty:
                    break

            # Flush audio queue
            while True:
                try:
                    self.audio_queue.get_nowait()
                except queue.Empty:
                    break

        self.get_logger().info('Text and audio queues flushed')

    def goal_callback(self, goal_request):
        '''
        Accepts or rejects a client request to begin an action.
        '''
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        '''
        Accepts or rejects a client request to cancel an action.
        '''
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """
        Callback function for the 'speech_output' service.
        """
        
        goal_msg = goal_handle.request
            # 1. Stop old thread
        self.stop_audio.set()
        self.it=0

        if hasattr(self, "audio_thread") and self.audio_thread.is_alive():
            self.audio_thread.join()

        # 2. Clear stop flag
        self.stop_audio.clear()

        # 3. Flush queues
        self.flush_queue()
        
        # # Start audio playback ONCE
        audio_thread = threading.Thread(
        target=self.audio_generator,
        args=(self.stop_audio,), 
        daemon=True
        )
        audio_thread.start()

        # Start Playback thread
        playback_thread = threading.Thread(
            target=self.playback_audio,
            daemon=True
        )
        playback_thread.start()
        self.flush_queue()
        self.is_playing = True
        
        if goal_msg.use_text_field:
            self.get_logger().info("Processing text from goal...")
            self.flush_queue()
            self.text_queue.put(goal_msg.text) 

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Cancel requested, stop audio...")
                    self.flush_queue()
                    self.stop_audio.set()
                    pygame.mixer.stop() 
                    audio_thread.join()
                    playback_thread.join()
                    goal_handle.canceled()
                    return output_action.Result(success=False)

                if self.text_queue.empty() and self.audio_queue.empty() and not self.is_playing and not self.is_generating:
                    self.get_logger().info("End of playback, stop audio...")
                    self.stop_audio.set()
                    audio_thread.join()
                    playback_thread.join()
                    goal_handle.succeed()
                    return output_action.Result(success=True)

                state = "playing"
                goal_handle.publish_feedback(
                    output_action.Feedback(state=state)
                )
                time.sleep(0.05)
        else:
            self.get_logger().info("Processing text from topic...")
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Cancel requested, stop audio...")
                    self.flush_queue()
                    self.stop_audio.set()
                    pygame.mixer.stop() 
                    audio_thread.join()
                    playback_thread.join()
                    goal_handle.canceled()
                    return output_action.Result(success=False)
                
                else:
                    with self.state_lock:
                        done = self.llm_done_event.is_set()
                        empty = self.text_queue.empty()
                        empty_audio = self.audio_queue.empty()

                    if done and empty and empty_audio and not self.is_playing and not self.is_generating:
                        self.get_logger().info("End of playback, stop audio...")
                        self.stop_audio.set()
                        audio_thread.join()
                        playback_thread.join()
                        goal_handle.succeed()
                        return output_action.Result(success=True)

                    # Feedback: idle or playing
                    state = "playing" if not self.text_queue.empty() else "idle"
                    goal_handle.publish_feedback(output_action.Feedback(state=state))
                    time.sleep(0.05)

    def text_callback(self, msg):
        '''
        Callback function for the '/output_text' topic.
        '''
        if msg.data == "LLM_DONE":
            self.llm_done_event.set()
            if self.it==0: # No answer and first response, no previous text sent meaning it is not already processing other texts, no overwrite bools
                self.get_logger().info("LLM empty response.")
                self.is_playing = False
                self.is_generating = False
                self.it=0
            self.get_logger().info("LLM done signal received.")
        else:
            self.it+=1
            if self.debug:
                self.get_logger().info(f"Add text to queue:\n {msg.data}")
            self.is_generating = True
            self.text_queue.put(msg.data)

def destroy_node(self):
    if self.piper:
        self.piper.terminate()
        self.piper.wait()
    super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    speech_output_server = SpeechOutputServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(speech_output_server)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        speech_output_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

