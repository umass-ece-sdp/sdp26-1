# Shared dictionary for multiprocessing
shared_dict = None

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