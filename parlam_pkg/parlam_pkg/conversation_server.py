from parlam_interfaces.action import Input, Llm, Output, Conversation
from std_srvs.srv import Empty
from std_msgs.msg import String, Bool, Int32
from rcl_interfaces.msg import Parameter
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
import threading
from string import Template
import random

import json
# import random
# import ast
# from parlam_pkg.chroma_database import ChromaDB
# from langchain.prompts import PromptTemplate
# from langchain_community.embeddings import HuggingFaceEmbeddings
# import os

# import warnings
# from langchain._api import LangChainDeprecationWarning
# warnings.simplefilter("ignore", category=LangChainDeprecationWarning)
# warnings.simplefilter("ignore", category=UserWarning)
# warnings.simplefilter("ignore", category=FutureWarning)
#warnings.simplefilter("ignore", category=UserWarning)
import time
import asyncio

#from parlam_pkg.exp_functions import get_new_file_name, write_conversations, delay_time

class ConversationServer(Node):

    def __init__(self):
        super().__init__('conversation_server',
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)  

        self._action_cb_group = ReentrantCallbackGroup() # MutuallyExclusiveCallbackGroup()
        self._topic_cb_group = ReentrantCallbackGroup()
        self._service_cb_group = ReentrantCallbackGroup()

        self.input_cancel_pending=False
        self.llm_cancel_pending=False
        self.output_cancel_pending=False

        self._timer = self.create_timer(0.1, self.timer_callback, callback_group=self._action_cb_group)

        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")
        # Declare parameters
        if not self.has_parameter("model"):
            self.get_logger().info("Declaring default value for model parameter...")
            self.declare_parameter("model", "mistral:7b")
        if not self.has_parameter("debug"):
            self.get_logger().info("Declaring default value for debug parameter...")
            self.declare_parameter("debug", False)
        if not self.has_parameter("documents_path"):
            self.get_logger().info("Declaring default value for data...")
            self.declare_parameter("documents_path", "")

        # Set parameters
        self.get_logger().info("LLM Model: " + self.get_parameter("model").value)
        self.get_logger().info("Interaction language: " + self.get_parameter("language").value) 
        self.get_logger().info("Conversation file directory: " + self.get_parameter("directory").value) 
        self.get_logger().info("Bool save conversation: " + str(self.get_parameter("save_conversation").value)) 
        self.get_logger().info("ID experiment: " + str(self.get_parameter("id_experiment").value)) 
        self.get_logger().info("Documents path: " + self.get_parameter("documents_path").value) 
        self.get_logger().info(f"Instructions: {self.get_parameter('instructions').value}")

        self.get_logger().info("\n --------- \n LOGS \n ---------")

        # Client Input
        self.input_client = ActionClient(self, Input, 'input_action')
        self.input_client.wait_for_server()

        # Client LLM
        self.llm_client = ActionClient(self, Llm, 'llm_action')
        self.llm_client.wait_for_server()

        # Client Output
        self.output_client = ActionClient(self, Output, 'output_action')
        self.output_client.wait_for_server()

        # Skip service
        self.srv_skip = self.create_service(Empty, 
                                            'skip_conversation', 
                                            self.skip_callback, 
                                            callback_group=self._service_cb_group)
        self.skip_lock = threading.Lock()

        # Output topic publisher
        self.text_publisher = self.create_publisher(
            String,
            '/output_text',
            1
        )

        # Action server for Behavior Tree
        self._action_server = ActionServer(
            self,
            Conversation,
            'conversation_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_cb_group
        )

        # Store active child goal handles
        self.current_input_goal = None
        self.current_llm_goal = None
        self.current_output_goal = None

        self._total_text=""

        # Import prompt
        with open(self.get_parameter("documents_path").value+"/prompt.txt", "r", encoding="utf-8") as f:
            self.system_prompt_template = f.read()
        #self.get_logger().info("prompt: \n\n"+self.system_prompt_template)

        # Import instructions
        # with open(self.get_parameter("documents_path").value+"/instructions.txt", "r", encoding="utf-8") as f:
        #     self.instructions_text = f.read()
        
        self.get_logger().info("Parlam conversation server is ready to receive inputs.")

    def timer_callback(self):
        if self.input_cancel_pending and self.current_input_goal is not None:
            self.get_logger().info("Cancelling input goal from timer callback")
            self.current_input_goal.cancel_goal_async()
            self.input_cancel_pending=False

        if self.llm_cancel_pending and self.current_llm_goal is not None:
            self.get_logger().info("Cancelling LLM goal from timer callback")
            self.current_llm_goal.cancel_goal_async()
            self.llm_cancel_pending=False

        if self.output_cancel_pending and self.current_output_goal is not None:
            self.get_logger().info("Cancelling Output goal from timer callback")
            self.current_output_goal.cancel_goal_async()
            self.output_cancel_pending=False

    async def execute_callback(self, goal_handle):
        # Write file conversations
        self.skipped_counter = 0
        self._active_goal_handle = goal_handle
        params = goal_handle.request.goal
        command = None
        text = None
        previous_questions_and_answers = []
        result = Conversation.Result()
        self.is_skipped = False
        previous_interaction = None

        # Check if empty goal
        if not params:
            self.get_logger().info("Received empty goal, start listening")
            command = 0
        else:
            for p in params:
                if p.name == "command":
                    command = p.value.integer_value
                elif p.name == "text":
                    text = p.value.string_value
                elif p.name == "history":
                    # Init chat history
                    try:
                        self.get_logger().error("Previous interaction parsed")
                        self.get_logger().error(f"Previous interaction raw: {p.value.string_value}")
                        previous_interaction = self.get_messages_history(previous_questions_and_answers=json.loads(p.value.string_value))
                    except Exception as e:
                        self.get_logger().error(f"Failed to parse history: {e}")
                        previous_interaction = ""

            # If command was not provided but goal wasn't empty
            if command is None:
                self.get_logger().info("No command provided, defaulting to LISTEN (0)")
                command = 0

        # Decide initial state
        if command == 0: #LISTEN
            state = "LISTEN"

        elif command == 1: #THINK AND ANSWER
            input = text
            state = "ANSWER"

        elif command == 2:
            if self.get_parameter("instructions").value: #Start with instructions
                self.publish_feedback(goal_handle=goal_handle, state_string="Speaking")
                await self.send_instructions(goal_handle, self.instructions_text)
                state = "LISTEN"
            else:
                text_to_speak= "You can start the task."
                state="SPEAK"
                
        else:
            goal_handle.abort()
            return Conversation.Result()
        
        # start_time = time.time()

        while rclpy.ok():

            if goal_handle.is_cancel_requested:
                if self.debug:
                    self.get_logger().info('Received cancel goal handle conversation server...')
                self.cancel_all_children()
                goal_handle.canceled()
                self.input_cancel_pending=False
                self.llm_cancel_pending=False
                self.output_cancel_pending=False
                self._active_goal_handle = None
                result_history = Parameter()
                result_history.name = "history"
                result_history.value.type = 4
                result_history.value.string_value = json.dumps(previous_questions_and_answers)
                result.result.append(result_history)
                self.get_logger().info('Ending goal as canceled...')
                return result
            
            with self.skip_lock:
                if self.is_skipped: 
                    state="LISTEN"
                    if self.debug:
                        self.get_logger().info("Conversation skipped, going back to listen state...")
                    self.is_skipped = False

            if state == "LISTEN":
                self.publish_feedback(goal_handle=goal_handle, state_string="Listening")
                if self.debug:
                    self.get_logger().info('Send goal to input server...')

                success, input = await self.handle_input(goal_handle)
                if not success:
                    if self.debug:
                        self.get_logger().info('Input goal not success...')
                    self.cancel_all_children()
                    goal_handle.canceled()
                    self._active_goal_handle = None
                    self.input_cancel_pending=False
                    self.llm_cancel_pending=False
                    self.output_cancel_pending=False
                    result_history = Parameter()
                    result_history.name = "history"
                    result_history.value.type = 4
                    result_history.value.string_value = json.dumps(previous_questions_and_answers)
                    result.result.append(result_history)
                    self.get_logger().info('Ending goal as canceled...')
                    return result
                if input == "offconv":
                    if self.debug:
                        self.get_logger().info("Received 'offconv' command, listening again...")
                    state = "LISTEN"
                else: #Looking and input received
                    if self.debug:
                        self.get_logger().info("Received input while looking, answering...")
                    self.publish_feedback(goal_handle=goal_handle, input_text=input)
                    state = "ANSWER"
                    
            elif state == "ANSWER":  
                self.publish_feedback(goal_handle=goal_handle, state_string="Answering")
                messages =self.get_messages_history(previous_questions_and_answers=previous_questions_and_answers)
                # self.get_logger().info("Messages: \n\n" + messages)
                
                final_input = self.build_ollama_messages(system_prompt=self.system_prompt_template,
                                                        previous_interaction=previous_interaction,
                                                        history=previous_questions_and_answers,
                                                        user_question=input)

                #self.get_logger().info("Messages: \n\n" + final_input)
                if self.debug:
                    self.get_logger().info('Send goal to llm and output servers...')
                success, final_text = await self.handle_think_and_output(goal_handle, 
                                                             model=self.get_parameter("model").value,
                                                             messages=final_input,
                                                             text = "",
                                                             use_text_field = False)
                self.get_logger().info(final_text)
                if not success:
                    if self.debug:
                        self.get_logger().info('Output and llm goals not success...')
                    self.cancel_all_children()
                    goal_handle.canceled()
                    self._active_goal_handle = None
                    self.input_cancel_pending=False
                    self.llm_cancel_pending=False
                    self.output_cancel_pending=False
                    result_history = Parameter()
                    result_history.name = "history"
                    result_history.value.type = 4
                    result_history.value.string_value = json.dumps(previous_questions_and_answers)
                    result.result.append(result_history)
                    self.get_logger().info('Ending goal as canceled...')
                    return result
                previous_questions_and_answers.append([input, final_text])
                state = "LISTEN"
            elif state == "SPEAK":
                self.publish_feedback(goal_handle=goal_handle, state_string="Speaking")
                if self.debug:
                    self.get_logger().info('Send goal to output server...')
                success = await self.handle_output(goal_handle,
                                                   text= text_to_speak,
                                                   use_text_field = True)
                if not success:
                    if self.debug:
                        self.get_logger().info('Output goal not success...')
                    self.cancel_all_children()
                    goal_handle.canceled()
                    self._active_goal_handle = None
                    self.input_cancel_pending=False
                    self.llm_cancel_pending=False
                    self.output_cancel_pending=False
                    result_history = Parameter()
                    result_history.name = "history"
                    result_history.value.type = 4
                    result_history.value.string_value = json.dumps(previous_questions_and_answers)
                    result.result.append(result_history)
                    self.get_logger().info('Ending goal as canceled...')
                    return result
                self.publish_feedback(goal_handle=goal_handle, output_text=text_to_speak)
                previous_questions_and_answers.append(["", text_to_speak])
                state = "LISTEN"
            # self.get_logger().info("Previous dialog: \n\n" + json.dumps(previous_questions_and_answers)   ) 
            time.sleep(0.01)

        goal_handle.succeed()
        self._active_goal_handle = None
        self.input_cancel_pending=False
        self.llm_cancel_pending=False
        self.output_cancel_pending=False
        result_history = Parameter()
        result_history.name = "history"
        result_history.value.type = 4
        result_history.value.string_value = json.dumps(previous_questions_and_answers)
        result.result.append(result_history)
        self.get_logger().info('Ending succeed goal...')
        return result

    def goal_callback(self, goal_request):
        '''
        Accepts or rejects a client request to begin an action.
        '''
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):

        self.get_logger().info("Received conversation cancel request")
        self.cancel_all_children()

        return CancelResponse.ACCEPT
    
    def cancel_all_children(self):

        self.input_cancel_pending=True
        self.llm_cancel_pending=True
        self.output_cancel_pending=True

        if self.current_input_goal is not None:
            self.get_logger().info("Cancelling input goal")
            self.current_input_goal.cancel_goal_async()
            self.input_cancel_pending=False

        if self.current_llm_goal is not None:
            self.get_logger().info("Cancelling LLM goal")
            self.current_llm_goal.cancel_goal_async()
            self.llm_cancel_pending=False

        if self.current_output_goal is not None:
            self.get_logger().info("Cancelling TTS goal")
            self.current_output_goal.cancel_goal_async()
            self.output_cancel_pending=False
    
    async def handle_input(self, goal_handle, listen_time=0):

        if goal_handle.is_cancel_requested:
            return False, ""

        input_goal = Input.Goal(listen_time=listen_time)

        self.current_input_goal = await self.input_client.send_goal_async(input_goal)

        if not self.current_input_goal.accepted:
            return False, ""

        result_future = await self.current_input_goal.get_result_async()
        self.current_input_goal = None

        if goal_handle.is_cancel_requested:
            return False, ""
        
        return True, result_future.result.user_input
    
    async def handle_output(self, goal_handle, text, use_text_field):
        """
        Trigger both LLM and Output goals concurrently.
        """
        if goal_handle.is_cancel_requested:
            return False, ""
        
        # Output
        output_goal = Output.Goal(text=text, use_text_field=use_text_field)
        self.current_output_goal = await self.output_client.send_goal_async(
            output_goal
        )

        if not self.current_output_goal.accepted:
            self.current_output_goal = None
            return False, ""
        
        await self.current_output_goal.get_result_async()
        self.current_output_goal = None

        if goal_handle.is_cancel_requested:
            return False, ""
        
        return True

    async def handle_think_and_output(self, goal_handle, model, messages, text, use_text_field):
        """
        Trigger both LLM and Output goals concurrently.
        """
        if goal_handle.is_cancel_requested:
            return False, ""
        
        # Output: send first goal to flush queue before LLM generated answers to add to queue
        output_goal = Output.Goal(text=text, use_text_field=use_text_field)
        self.current_output_goal = await self.output_client.send_goal_async(
            output_goal
        )

        if not self.current_output_goal.accepted:
            self.current_output_goal = None
            return False, ""

        # LLM
        llm_goal = Llm.Goal(model=model, messages=messages) #TODO: change it to prompt
        self.current_llm_goal = await self.llm_client.send_goal_async(
            llm_goal,
            feedback_callback=self.llm_feedback_callback
        )

        if not self.current_llm_goal.accepted:
            self.current_llm_goal = None
            return False, ""
        
        if goal_handle.is_cancel_requested:
            return False, ""

        # Wait LLM response
        llm_result_future = await self.current_llm_goal.get_result_async()
        self.current_llm_goal = None

        if self.debug:
            self.get_logger().info('Future llm goal finished')

        if goal_handle.is_cancel_requested:
            return False, ""
        
        await self.current_output_goal.get_result_async()
        self.current_output_goal = None

        if self.debug:
            self.get_logger().info('Future output goal finished')
        
        if goal_handle.is_cancel_requested:
            return False, ""
        
        return True, llm_result_future.result.final_text

    def llm_feedback_callback(self, feedback_msg):
        """
        Called every time the LLM action server sends partial feedback. Depending on the use case it can be answer or a flag defined to trigger robot actions.
        """
        partial_text = feedback_msg.feedback.partial_text

        self.get_logger().info(f"LLM feedback: {partial_text}")

        # Publish to TTS topic
        self.text_publisher.publish(String(data=partial_text))

        if self._active_goal_handle is not None and partial_text!="LLM_DONE":

            self._total_text += partial_text
            self.publish_feedback(goal_handle=self._active_goal_handle, output_text=self._total_text)
        if partial_text=="LLM_DONE":
            self._total_text=""

        ############################################################################################################################
        # Example of output with flags in the text 
        # if self.get_parameter("system").value==2:

        #     # Process streaming LLM output
        #     if partial_text.startswith("SPEECH:"):
        #         # Extract speech text
        #         speech_text = partial_text[len("SPEECH:"):].strip()
        #         # Publish to TTS topic
        #         self.text_publisher.publish(String(data=speech_text))
        #         # Add to total text and publish feedback
        #         if self._active_goal_handle is not None:
        #             self._total_text += speech_text
        #             self.publish_feedback(goal_handle=self._active_goal_handle, output_text=self._total_text)

        #     elif partial_text.startswith("MOVE_TO_PERSON:"):
        #         # Extract move value safely
        #         if self.debug:
        #             self.get_logger().info(f"Received move command feedback: {partial_text}")
        #         try:
        #             move_value = int(partial_text.split("MOVE_TO_PERSON:")[1].strip()[0])
                    
        #         except (IndexError, ValueError):
        #             move_value = 0

        #         move_bool = bool(move_value)
        #         if self.debug:
        #                 self.get_logger().info(f"Converted successfully move value to bool: {move_bool}")
        #         # Publish move feedback
        #         if self._active_goal_handle is not None:
        #             self.publish_feedback(goal_handle=self._active_goal_handle, move_value=move_bool)

        #     elif partial_text == "LLM_DONE":
        #         # Reset total text or perform any finalization
        #         self.text_publisher.publish(String(data=partial_text))
        #         self._total_text = ""

        #     else:
        #         # Any other text is treated as speech fallback
        #         self.text_publisher.publish(String(data=partial_text))
        #         if self._active_goal_handle is not None:
        #             self._total_text += partial_text
        #             self.publish_feedback(goal_handle=self._active_goal_handle, output_text=self._total_text)
        ############################################################################################################################

    def build_ollama_messages(self,     
                              system_prompt: str,
                              previous_interaction: str,
                              history: list,
                              user_question: str
                            ) -> str:
        """Returns a JSON string of the messages list."""
        
        full_system = f"{system_prompt}\n\nPREVIOUS INTERACTION:\n{previous_interaction}"
        
        messages = [{"role": "system", "content": full_system}]
        for user_msg, assistant_msg in history:
            messages.append({"role": "user",      "content": user_msg})
            messages.append({"role": "assistant", "content": assistant_msg})
        messages.append({"role": "user", "content": user_question})
            
        return json.dumps(messages)


    def skip_callback(self, request, response):
        self.skipped_counter+= 1
        #if self.debug:
        self.get_logger().info("Received skip service request, cancelling all goals...")
        self.cancel_all_children()
        self.input_cancel_pending = False
        self.output_cancel_pending = False
        self.llm_cancel_pending = False
        self.is_skipped = True
        return response
    
    # async def send_instructions(self,goal_handle, text):

    #     paragraphs = [p.strip() for p in text.split("\n\n") if p.strip()]

    #     for paragraph in paragraphs:
    #         if self.debug:
    #             self.get_logger().info(f"Sending instructions paragraph: {paragraph}")
    #         success = await self.handle_output(goal_handle,
    #                                 text= paragraph,
    #                                 use_text_field = True)
    #         #await asyncio.sleep(3)
    
    def publish_feedback(
        self,
        goal_handle,
        state_string: str = None,
        input_text: str = None,
        output_text: str = None,
        move_value: bool = None
    ):
        
        feedback_msg = Conversation.Feedback()
        feedback_msg.feedback = []

        if state_string is not None:
            state_msg = Parameter()
            state_msg.name = "feedback_state"
            state_msg.value.type = 4
            state_msg.value.string_value = state_string
            feedback_msg.feedback.append(state_msg)

        if input_text is not None:
            input_dialog = Parameter()
            input_dialog.name = "input_dialog"
            input_dialog.value.type = 4
            input_dialog.value.string_value = input_text
            feedback_msg.feedback.append(input_dialog)

        if output_text is not None:
            output_dialog = Parameter()
            output_dialog.name = "output_dialog"
            output_dialog.value.type = 4
            output_dialog.value.string_value = output_text
            feedback_msg.feedback.append(output_dialog)
        
        # Adapt as needed depending on the flags

        # if move_value is not None:
        #     move_msg = Parameter()
        #     move_msg.name = "go_to_person"
        #     move_msg.value.type = 1
        #     move_msg.value.bool_value = move_value
        #     feedback_msg.feedback.append(move_msg)

        # Only publish if something was actually added
        if feedback_msg.feedback:
            goal_handle.publish_feedback(feedback_msg)

    def get_messages_history(self, previous_questions_and_answers):
        """Get the messages prompt, including system prompt, previous interaction and new input.

        Args:
            system_prompt: The instructions for the chat bot - this determines how it will behave.
            context_data: Relevant context information to answer the question.
            previous_questions_and_answers: Chat history.
            new_question: The new question to ask the bot.

        Returns:
            The messages text to pass to the LLM.
        """

        messages = ""
        for question, answer in previous_questions_and_answers:
                messages += "Question: " + question + "\n"
                messages +=  "Answer: " + answer + "\n"
        return messages

def main(args=None):
    rclpy.init(args=args)

    conversation = ConversationServer()
    executor = MultiThreadedExecutor(4)
    executor.add_node(conversation)
    executor.spin()

    conversation.get_logger().info('Destroying node...')
    conversation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

