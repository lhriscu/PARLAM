import os

import numpy as np


def get_new_file_name(directory, id, language, task, type):
    """
        Generate a new file name based on existing files in the directory.  
        The new file name will have an incremented id_person part.  

        Args:
            directory (str): The directory to search for existing files.
            id (str): The id part of the file name.
            language (str): The language part of the file name.
            task (str): The task part of the file name.
            type (str): The type part of the file name. 
        Returns:
            str: The new file name with an incremented id_person part.
    """
    max_id = -1
    for file_name in os.listdir(directory):
        parts = file_name.split("_")
        
        if len(parts) != 5:
            # Skip files that do not follow the naming structure
            continue

        id_name, language_name, task_name, type_name, id_person_exp = parts

        if task_name == task and type_name == type and language_name == language and id_name==id:
            try:
                # Convert the id_person part to an integer
                id_person_exp = int(id_person_exp)

                # Update the max_id if this one is larger
                if id_person_exp > max_id:
                    max_id = id_person_exp
            except ValueError:
                # Skip files where id_person is not an integer
                continue


    new_id = max_id + 1
    new_file_name = f"{id}_{language}_{task}_{type}_{new_id}"
    return new_file_name


def delay_time(spc, nchar):
    """
    Calculate a delay time based on seconds per character and number of characters.

    Args:
        spc (float): Seconds per character input.
        nchar (int): Number of characters answer.      
    Returns:
        float: Calculated delay time in seconds.
    """
    delay = 1 + np.random.normal(loc=spc, scale=spc*0.1)*nchar 
    return delay

def write_conversations(file_path, total_duration, avg_input_time_stt, avg_reply_time_llm,avg_reply_time_tts, context_data, conversation_data):
    """ 
    Write conversation data along with context and timing information to a file.
    Args:
        file_path (str): The path to the file where the conversation will be written.
        total_duration (float): Total duration of the conversation.
        avg_input_time_stt (float): Average input time for speech-to-text.
        avg_reply_time_llm (float): Average reply time for language model.
        avg_reply_time_tts (float): Average reply time for text-to-speech.
        context_data (list): List of context strings corresponding to each user-assistant pair.
        conversation_data (list): List of tuples containing user and assistant dialog.
    Returns:
        bool: True if writing was successful, otherwise an error message.   
    """  
    full_data = [tuple1 + (value,) for tuple1, value in zip(conversation_data[1:], context_data)]
    # First pair of conversation_data is ('', greeting assistant)
    try:
        with open(file_path, "a+") as file:
            # Iterate over each (user, assistant) pair
            file.write(f"Total duration: {total_duration:.2f} seconds\n")
            file.write(f"Average input time STT: {avg_input_time_stt:.2f} seconds\n")
            file.write(f"Average replay time LLM: {avg_reply_time_llm:.2f} seconds\n")
            file.write(f"Average replay time LLM+TTS: {avg_reply_time_tts:.2f} seconds\n")
            file.write(f"Assistant: {conversation_data[0][1]}\n")
            for user_text, assistant_text, context in full_data:
                # Write user dialog
                file.write(f"User: {user_text}\n")
                # Context
                file.write(f"Context: {context}\n")
                # Write assistant dialog
                file.write(f"Assistant: {assistant_text}\n")
        return True
    except ValueError as e:
        return f"Error: {e}"
                
