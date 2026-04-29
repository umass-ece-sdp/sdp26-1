import threading

# Shared state across threads. Module-level globals are shared between
# threads (same process, same interpreter) but NOT between subprocesses.

_lock = threading.Lock()
instruction = {}
glove_connected = False
drone_connected = False

# Event used by the launcher to signal a clean shutdown to all threads.
# Threads should call shutdown_event.is_set() and exit promptly when true.
shutdown_event = threading.Event()

# Set by the glove client whenever the dict has been freshly populated.
# The drone thread waits on this rather than busy-spinning on
# `glove_connected` (which had no GIL release point and pegged a CPU core).
glove_ready_event = threading.Event()


def write_instr(instr: tuple):
    """
    Write instruction to shared variable.

    Parameters:
        instr: (fingers_4tuple, speed_float, dist_float)
    """
    global instruction
    with _lock:
        instruction['fingers'] = instr[0]
        instruction['speed'] = instr[1]
        instruction['dist'] = instr[2]
    glove_ready_event.set()


def read_instr() -> dict:
    """Read a snapshot of the current instruction (thread-safe)."""
    with _lock:
        return dict(instruction)


def set_glove_on():
    global glove_connected
    glove_connected = True
    print('Glove connected and set to on')


def set_glove_off():
    global glove_connected
    glove_connected = False
    glove_ready_event.clear()
    print('Glove disconnected and set to off')


def set_drone_on():
    global drone_connected
    drone_connected = True
    print('Drone connected and set to on')


def set_drone_off():
    global drone_connected
    drone_connected = False
    print('Drone disconnected and set to off')


def request_shutdown():
    """Signal all threads to exit cleanly."""
    shutdown_event.set()
