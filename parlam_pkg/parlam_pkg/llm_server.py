#!/usr/bin/env python3
import os
import ast

from parlam_interfaces.srv import Llm

import rclpy
from rclpy.node import Node

from openai import OpenAI
from langchain.prompts import PromptTemplate
from dotenv import load_dotenv


class LlmServer(Node):

    def __init__(self):
        """
        Initializes the LlmServer node.
        """
        super().__init__('llm_server',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        self.srv = self.create_service(Llm, 'llm', self.llm_callback)

        load_dotenv()
        key=os.environ['OPENAI_API_KEY']

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        self.get_logger().info("LLM Service is ready to receive inputs.")  

    def get_response(self, input):
        """
        Sends a chat message to the OpenAI API and returns the generated response.

        Args:
            input (list): A list of message dictionaries formatted as expected by the OpenAI Chat API.
                        Each dictionary should contain keys like 'role' (e.g., 'user', 'system') and 'content'.

        Returns:
            str: The content of the model's generated response.
        """
        client = OpenAI()
        completion = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=input,
            temperature=1,
            max_tokens=1000
        )
        return completion.choices[0].message.content

    def llm_callback(self, request, response):
        """
        Callback function for the 'llm" service.
        """
        self.get_logger().info("Obtaining LLM answer...")
        response.answer = self.get_response(ast.literal_eval(request.messages)) #Passed as a string but needs to be processed as a list
        response.success = True
        response.message = ""
        return response

def main(args=None):
    rclpy.init(args=args)

    llm_server = LlmServer()

    rclpy.spin(llm_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

