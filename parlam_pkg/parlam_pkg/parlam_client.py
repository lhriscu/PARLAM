from parlam_interfaces.srv import Input, Llm, Output
from parlam_interfaces.action import Llm as llm_action
from rcl_interfaces.msg import Parameter
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse

import json
import random
import ast
from openai import OpenAI
from parlam_pkg.chroma_database import ChromaDB
from langchain.prompts import PromptTemplate
from langchain_community.embeddings import HuggingFaceEmbeddings
import os
import time
import re

import warnings
from langchain._api import LangChainDeprecationWarning
warnings.simplefilter("ignore", category=LangChainDeprecationWarning)
warnings.simplefilter("ignore", category=UserWarning)
warnings.simplefilter("ignore", category=FutureWarning)
#warnings.simplefilter("ignore", category=UserWarning)

from parlam_pkg.exp_functions import get_new_file_name, write_conversations, delay_time

class ParlamClientAsync(Node):

    def __init__(self):
        super().__init__('parlam_client_async',
                         allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)  

        # Set parameters
        self.get_logger().info("\n --------- \n PARAMETERS \n ---------")
        self.get_logger().info("Prompt path: " + self.get_parameter("prompt_path").value) 
        self.get_logger().info("Document Chromadb path: " + self.get_parameter("document_chromadb_path").value) 
        self.get_logger().info("Folder Chromadb path: " + self.get_parameter("folder_chromadb_path").value) 
        self.get_logger().info("Greetings file path: " + self.get_parameter("greetings_path").value) 
        self.get_logger().info("Interaction language: " + self.get_parameter("language").value) 
        self.get_logger().info("Task name: " + self.get_parameter("task").value) 
        self.get_logger().info("Interaction type: " + self.get_parameter("type").value) 
        self.get_logger().info("Conversation file directory: " + self.get_parameter("directory").value) 
        self.get_logger().info("Bool save conversation: " + str(self.get_parameter("save_conversation").value)) 
        self.get_logger().info("ID experiment " + str(self.get_parameter("id_experiment").value)) 
        
        # Client Input
        self.cli_input = self.create_client(Input, 'speech_input')
        while not self.cli_input.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Input service not available, waiting again...')
        self.req_input = Input.Request()

        # Client LLM
        self.cli_llm = self.create_client(Llm, 'llm')
        while not self.cli_llm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LLM service not available, waiting again...')
        self.req_llm = Llm.Request()

        # Client Output
        self.cli_output = self.create_client(Output, 'speech_output')
        while not self.cli_output.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Output service not available, waiting again...')
        self.req_output = Output.Request()

        self.llm_action_server = ActionServer(
            self,
            llm_action,
            'llm_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)        

        # Get prompt template
        with open(self.get_parameter("prompt_path").value + '/' + self.get_parameter("task").value +'.txt', "r") as file: 
            system_prompt_file= file.read().strip()

        # Get greetings
        with open(self.get_parameter("greetings_path").value, "r", encoding="utf-8") as file:
            self.greetings = json.load(file)

        self.system_prompt_template = PromptTemplate.from_template(template=system_prompt_file)

        if self.get_parameter("task").value == "info":
            # Get or create Chroma Database
            embeddings = HuggingFaceEmbeddings(model_name="sentence-transformers/LaBSE")
            
            self.chroma_database = ChromaDB(doc_path=self.get_parameter("document_chromadb_path").value, 
                                            db_path=self.get_parameter("folder_chromadb_path").value,
                                            embeddings=embeddings)
            
            self.get_logger().info("Chroma database is loaded.")

        # Init context vector
        self.context_list = [] # Remains empty for handover tasks

        self.get_logger().info("\n --------- \n LOGS \n ---------")
        self.get_logger().info("Info dialog LLM is ready to receive inputs.")


    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        
        # goal = llm_action.Goal()
        # goal = goal_handle.request
        self.get_logger().info('Executing goal...')

        # Init chat history
        try:
            previous_questions_and_answers = ast.literal_eval(self.goal.history)
        except Exception as e:
            self.get_logger().error(f"Failed to parse history: {e}")
            previous_questions_and_answers = []

        # Initialize feedback message
        feedback_msg = llm_action.Feedback()

        partial_dialog = Parameter()
        partial_dialog.name="partial_dialog"
        partial_dialog.value.type = 4 #string
        partial_dialog.value.string_value = self.goal.history

        state = Parameter()
        state.name = "state"
        state.value.type = 4
        state.value.string_value = "Start"

        input_dialog = Parameter() 
        input_dialog.name="input_dialog"
        input_dialog.value.type = 4 #string
        input_dialog.value.string_value = 'default'

        output_dialog = Parameter() 
        output_dialog.name="output_dialog"
        output_dialog.value.type = 4 #string
        output_dialog.value.string_value = 'default'

        handover_param = Parameter()
        handover_param.name="bool_handover"
        handover_param.value.type=1 # bool
        handover_param.value.bool_value=False

        # Initialize input response message
        response_input= Input.Response()
        response_input.user_input = "default"

        context_data = ''


        if not previous_questions_and_answers:
          # Send greeting 
          self.get_logger().info("Preparing to say greeting.")
          greeting = random.choice(self.greetings[self.get_parameter("language").value])
          await self.send_request_output(answer=greeting)
          previous_questions_and_answers.append(("", greeting))
        
        else:
            self.get_logger().info("Previous questions and answers found, continuing conversation.")

        if self.get_parameter("task").value == "info":

            while not goal_handle.is_cancel_requested: 
                # Listen input
                state.value.string_value = "Listening"
                feedback_msg.feedback.append(state)
                goal_handle.publish_feedback(feedback_msg)

                # Listen input
                response_input = await self.send_request_input()

                if response_input and response_input.user_input!='offconv':

                    input_dialog.value.string_value = response_input.user_input
                    feedback_msg.feedback.append(input_dialog)
                    goal_handle.publish_feedback(feedback_msg)

                    # Get messages with input question
                    context_data, chunks = self.get_context(response_input.user_input)
                    messages = self.get_messages_info(context_data=context_data,
                                                    previous_questions_and_answers=previous_questions_and_answers,
                                                    new_question= response_input.user_input)
                            
                    # Get LLM answer
                    response_llm = await self.send_request_llm(messages=str(messages)) #Send as a string for interface communication data type
                    self.get_logger().info('LLM output: ' + (response_llm.answer))
                    
                    if response_llm.answer!= '' and response_llm.answer != 'end':
                        # Play output
                        state.value.string_value = "Speaking"
                        output_dialog.value.string_value = response_llm.answer
                        feedback_msg.feedback.append(output_dialog)
                        feedback_msg.feedback.append(state)
                        goal_handle.publish_feedback(feedback_msg)

                        # Delay time to mimic human typing
                        #time.sleep(delay_time(0.3, len(str(response_llm.answer))))

                        await self.send_request_output(answer=str(response_llm.answer))
                                            # Update chat history
                    
                        previous_questions_and_answers.append((str(response_input.user_input), str(response_llm.answer)))
                        self.context_list.append((str(context_data)))

                        partial_dialog.value.string_value = str(previous_questions_and_answers)
                        feedback_msg.feedback.append(partial_dialog)
                        goal_handle.publish_feedback(feedback_msg)

                    else: 
                        break
        
            if feedback_msg.feedback:
                # Clear feedback messages
                self.get_logger().info('Clearing feedback messages.')
                feedback_msg.feedback.clear()

            result = llm_action.Result()
            if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return result
            else:
                goal_handle.succeed()
                param = Parameter()
                result.result.append(param)

                return result

        elif self.get_parameter("task").value == "handover":

            while not goal_handle.is_cancel_requested: 
                # Listen input
                state.value.string_value = "Listening"
                feedback_msg.feedback.append(state)
                goal_handle.publish_feedback(feedback_msg)

                # Listen input
                response_input = await self.send_request_input()

                if response_input and response_input.user_input!='offconv':

                    input_dialog.value.string_value = response_input.user_input
                    feedback_msg.feedback.append(input_dialog)
                    goal_handle.publish_feedback(feedback_msg)

                    # Get messages with input question
                    messages = self.get_messages_handover(previous_questions_and_answers=previous_questions_and_answers, 
                                                        new_question= response_input.user_input)

                    # Get LLM answer
                    response_llm = await self.send_request_llm(messages=str(messages)) #Send as a string for interface communication data type
                    self.get_logger().info('LLM output: ' + (response_llm.answer))
                if 'pick_up' in str(response_llm.answer).lower():
                    handover_param.value.bool_value = True
                    feedback_msg.feedback.append(handover_param)
                    goal_handle.publish_feedback(feedback_msg)

                elif response_llm.answer!= '' and response_llm.answer != 'end':
                    # Play output
                    state.value.string_value = "Speaking"
                    output_dialog.value.string_value = response_llm.answer
                    feedback_msg.feedback.append(output_dialog)
                    feedback_msg.feedback.append(state)
                    goal_handle.publish_feedback(feedback_msg)

                    # Delay time to mimic human typing
                    #time.sleep(delay_time(0.3, len(str(response_llm.answer))))

                    await self.send_request_output(answer=str(response_llm.answer))
                                        # Update chat history
                
                    previous_questions_and_answers.append((str(response_input.user_input), str(response_llm.answer)))
                    self.context_list.append((str(context_data)))

                    partial_dialog.value.string_value = str(previous_questions_and_answers)
                    feedback_msg.feedback.append(partial_dialog)
                    goal_handle.publish_feedback(feedback_msg)
                
                else:
                    break

            if feedback_msg.feedback:
                # Clear feedback messages
                self.get_logger().info('Clearing feedback messages.')
                feedback_msg.feedback.clear()

            result = llm_action.Result()
            if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return result
            else:
                goal_handle.succeed()
                param = Parameter()
                param.name="handover_done"
                param.value.type=1 # bool
                param.value.bool_value=handover_param.value.bool_value
                result.result.append(param)

                return result


        if self.get_parameter("save_conversation").value:
            os.makedirs(self.get_parameter("directory").value, exist_ok=True)          
            file_name = get_new_file_name(directory=self.get_parameter("directory").value,
                                          id=self.get_parameter("id_experiment").value,
                                          language=self.get_parameter("language").value,
                                          task=self.get_parameter("task").value,
                                          type=self.get_parameter("type").value
                                          )
            self.get_logger().info('New conversation file: ' + (file_name))
            # Average times are set to 'default' as placeholders, have to be computed during the conversation
            write_conversations(file_path=self.get_parameter("directory").value + '/'+file_name,
                                total_duration='default',
                                avg_input_time_stt = 'default',
                                avg_reply_time_llm='default',
                                avg_reply_time_tts='default',
                                context_data=self.context_list,
                                conversation_data=previous_questions_and_answers)
            self.get_logger().info('New file saved')

        goal_handle.succeed()

        param = Parameter()

        return result

    def get_messages_info(self, context_data, previous_questions_and_answers, new_question):
        """Get the messages text, a prompt formatted with context data for RAG, new question and previous chat history.

        Args:
            system_prompt: The instructions for the chat bot - this determines how it will behave.
            context_data: Relevant context information to answer the question.
            previous_questions_and_answers: Chat history.
            new_question: The new question to ask the bot.

        Returns:
            The messages text to pass to the LLM.
        """

        system_prompt_formatted: str = self.system_prompt_template.format(
            context_data=context_data,
            language=self.get_parameter("language").value)
    
        # Build the messages with system prompt instructions
        messages = [
            { "role": "system", "content": system_prompt_formatted }
        ]
        
        # Add the previous questions and answers
        for question, answer in previous_questions_and_answers:
            messages.append({ "role": "user", "content": question })
            messages.append({ "role": "assistant", "content": answer })
        # Add the new question
        messages.append({ "role": "user", "content": new_question })
        #messages.append({"role":"assistant", "content": ""})

        return messages
    
    def get_messages_handover(self, previous_questions_and_answers, new_question):
        """Get the messages text, a prompt formatted with new question and previous chat history.

        Args:
            system_prompt: The instructions for the chat bot - this determines how it will behave.
            previous_questions_and_answers: Chat history.
            new_question: The new question to ask the bot.

        Returns:
            The messages text to pass to the LLM.
        """

        system_prompt_formatted: str = self.system_prompt_template.format(
            language=self.get_parameter("language").value)
    
        # Build the messages with system prompt instructions
        messages = [
            { "role": "system", "content": system_prompt_formatted }
        ]
        
        # Add the previous questions and answers
        for question, answer in previous_questions_and_answers:
            messages.append({ "role": "user", "content": question })
            messages.append({ "role": "assistant", "content": answer })
        # Add the new question
        messages.append({ "role": "user", "content": new_question })
        #messages.append({"role":"assistant", "content": ""})

        return messages

    def get_context(self, question, k=8):
        #context = self.chroma_database.chroma_database().similarity_search(question, k)
        chunks, similaritydb = self.chroma_database.chroma_database()
        context = similaritydb.similarity_search(question,k)
        #self.get_logger().info(str(chunks))        
        context_text = ''.join(i.page_content for i in context)
        #cleaned_text = re.sub(r"(?<!\n)\n(?!\n)", " ", context_text)  # Single newlines -> space
        #cleaned_text = re.sub(r"\n\n+", "\n\n", cleaned_text)  # Excessive newlines -> one
        return context_text, chunks
    
    async def send_request_input(self):
        self.future_input = self.cli_input.call_async(self.req_input)
        #rclpy.spin_until_future_complete(self, self.future_input)
        await self.future_input
        result = self.future_input.result()
        if result is None:
            self.get_logger().error("Input service call failed.")
        return result
    
    async def send_request_llm(self, messages):
        self.req_llm.messages =  messages
        self.future_llm = self.cli_llm.call_async(self.req_llm)
        #rclpy.spin_until_future_complete(self, self.future_llm)
        await self.future_llm
        result = self.future_llm.result()
        if result is None:
            self.get_logger().error("LLM service call failed.")
        return result

    async def send_request_output(self, answer):
        self.req_output.answer =  answer
        self.future_output = self.cli_output.call_async(self.req_output)
        #rclpy.spin_until_future_complete(self, self.future_output)
        await self.future_output
        result = self.future_output.result()
        if result is None:
            self.get_logger().error("Output service call failed.")
        return result
    

def main(args=None):
    rclpy.init(args=args)

    parlam_client_async = ParlamClientAsync()
    rclpy.spin(parlam_client_async)
    parlam_client_async.get_logger().info('Destroying node...')
    parlam_client_async.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
