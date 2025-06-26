import os

import numpy as np


def get_new_file_name(directory, id, language, task, type):
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
    # spc: seconds per char human
    # nchar: number of chars answer
    delay = 1 + np.random.normal(loc=spc, scale=spc*0.1)*nchar 
    return delay

def write_conversations(file_path, total_duration, avg_input_time_stt, avg_reply_time_llm,avg_reply_time_tts, context_data, conversation_data):
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
                
