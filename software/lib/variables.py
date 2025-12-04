# Shared instruction string for multiprocessing
instruction = ''
color = ''
glove_connected = False
drone_connected = False

def write_instr(instr: str):
    '''Write instruction to shared variable'''
    global instruction
    instruction = instr

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

def set_color(c: str):
    global color
    color_map = {
        'red': '#a61919',
        'orange': '#F98C1E',
        'green': '#75F91E',
        'yellow': '#F6FA00',
        'pink': '#FA00D5',
    }

    color = color_map[c.lower().strip()]