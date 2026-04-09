import threading

# Shared instruction string for multiprocessing
_lock = threading.Lock()
instruction = {}
color = ''
glove_connected = False
drone_connected = False

def write_instr(instr: tuple[tuple, tuple, tuple]):
    '''Write instruction to shared variable'''
    global instruction
    with _lock:
        instruction['fingers'] = instr[0]
        instruction['speed'] = instr[1]
        instruction['dist'] = instr[2]

def read_instr() -> dict:
    '''Read a snapshot of the current instruction (thread-safe)'''
    with _lock:
        return dict(instruction)

def set_glove_on():
    global glove_connected
    glove_connected = True
    print('Glove connected and set to on')

def set_glove_off():
    global glove_connected
    glove_connected = False
    print('Glove disconnected and set to off')

def set_drone_on():
    global drone_connected
    drone_connected = True
    print('Drone connected and set to on')

def set_drone_off():
    global drone_connected
    drone_connected = False
    print('Drone disconnected and set to off')