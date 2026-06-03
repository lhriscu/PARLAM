#!/usr/bin/env python3
#from librosa import stream

import rclpy
from rclpy.node import Node
import requests
from rclpy.executors import MultiThreadedExecutor
from parlam_interfaces.action import Llm as llm_action
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
import json
from std_msgs.msg import String
import threading
import time
import re

class LlmServer(Node):
    """
    Initializes the LlmServer node.
    """
    def __init__(self):
        super().__init__('llm_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)
        
        self._action_server = ActionServer(
            self,
            llm_action,
            'llm_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        # self.text_publisher = self.create_publisher(
        #     String,
        #     '/output_text',
        #     10
        # )

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        self.get_logger().info("LLM Service is ready to receive inputs.")  

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
        return CancelResponse.ACCEPT
    
def execute_callback(self, goal_handle):
        goal_msg = goal_handle.request
        final_text = ""
        feedback = llm_action.Feedback()  # assuming your action has a Feedback message
        # partial_response = ""
        speech_buffer = ""
        result = llm_action.Result()  # assuming your action has a Result message
        parsed = json.loads(goal_msg.messages)

        # # Validate structure
        # assert isinstance(parsed, list), "messages must be a list"
        # for i, msg in enumerate(parsed):
        #     assert isinstance(msg, dict),          f"message {i} is not a dict: {msg}"
        #     assert "role" in msg,                  f"message {i} missing 'role'"
        #     assert "content" in msg,               f"message {i} missing 'content'"
        #     assert isinstance(msg["content"], str), f"message {i} content is not a string"

        # self.get_logger().info("Message structure validated OK")

        def stream_llm():
            nonlocal final_text, speech_buffer
            
            try:
                with requests.post(
                    "http://IP/api/chat",
                    json={
                        "model": goal_msg.model,
                        "messages": json.loads(goal_msg.messages),
                        "options": {"temperature": 0.2, "num_predict": 1000},
                        "stream": True,
                        "top_p": 0.9
                    },
                    stream=True
                ) as r:
                    #self.get_logger().info(f"Ollama response status: {r.status_code}")
                    
                    if r.status_code != 200:
                        self.get_logger().error(f"Ollama error: {r.status_code} - {r.text}")
                        goal_handle.abort()
                        result.success = False
                        result.final_text = ""
                        return

                    for line in r.iter_lines():

                        if goal_handle.is_cancel_requested:
                            r.close()
                            goal_handle.canceled()
                            result.success = False
                            result.final_text = final_text
                            return

                        if not line:
                            continue

                        data = json.loads(line)
                        chunk = data.get("message", {}).get("content", "")
                        is_done = data.get("done", False)

                        speech_buffer += chunk
                        # self.get_logger().info(f"Received chunk: {chunk}")
                        # self.get_logger().info(f"Current speech buffer: {speech_buffer}")

                        if is_done:
                            self.get_logger().info("Received done signal, flushing buffer...")
                            if speech_buffer.strip():
                                final_text += speech_buffer.strip()
                                speech_buffer = ""

                            # SET RULES FOR FLAGS, IF ANY
                            goal_handle.publish_feedback(feedback)
                            goal_handle.succeed()
                            result.success = True
                            result.final_text = final_text
                            return

                        if speech_buffer.endswith(('.', '!', '?', '\n')):
                            self.get_logger().info("Sentence boundary flush")
                            speech_text = speech_buffer.strip()
                            if speech_text:
                                feedback.partial_text = speech_text
                                goal_handle.publish_feedback(feedback)
                                final_text += speech_text
                            speech_buffer = ""

                # If we exit the loop without hitting is_done
                self.get_logger().error("Stream ended without done signal")
                goal_handle.abort()
                result.success = False
                result.final_text = final_text

            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON decode error: {e}")
                goal_handle.abort()
                result.success = False
                result.final_text = final_text

            except requests.exceptions.RequestException as e:
                self.get_logger().error(f"Request error: {e}")
                goal_handle.abort()
                result.success = False
                result.final_text = final_text

            except Exception as e:
                self.get_logger().error(f"Unexpected error in stream_llm: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                goal_handle.abort()
                result.success = False
                result.final_text = final_text

        # Run streaming in a separate thread to avoid blocking ROS2 executor
        thread = threading.Thread(target=stream_llm)
        thread.start()
        thread.join()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = LlmServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f'An error occurred: {e}')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
