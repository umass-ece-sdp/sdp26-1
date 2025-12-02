# Shared instruction string for multiprocessing
instruction = ''
glove_connected = False

def write_instr(instr: str):
    '''Write instruction to shared variable'''
    global instruction
    instruction = instr

def set_glove_on():
    global glove_connected
    glove_connected = True

def set_glove_off():
    global glove_connected
    glove_connected = False