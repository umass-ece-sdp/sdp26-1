# Shared dictionary for multiprocessing
shared_dict = None
glove_connected = False

def init_shared(manager_dict):
    '''Initialize shared variables for multiprocessing'''
    global shared_dict
    shared_dict = manager_dict
    if 'instruction' not in shared_dict:
        shared_dict['instruction'] = ''

def get_instr():
    '''Get instruction from shared dict'''
    if shared_dict is not None:
        return shared_dict.get('instruction', '')
    return ''

def write_instr(instr: str):
    '''Write instruction to shared dict'''
    if shared_dict is not None:
        shared_dict['instruction'] = instr

def set_glove_on():
    global glove_connected
    glove_connected = True

def set_glove_off():
    global glove_connected
    glove_connected = False