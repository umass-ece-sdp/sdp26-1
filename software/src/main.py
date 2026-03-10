from hardware.firmware import server
from multiprocessing import Process, Pipe, Event
import socket, os, subprocess
from pathlib import Path
from software.lib.falcon import FALCON

def server_process_func(conn: socket.socket, sock: socket.socket, pipe_send):
    '''Process function to run the server (pinned to core 0)'''
    print('Starting server process...')
    os.sched_setaffinity(0, {0})
    server.run_server(conn, sock, pipe_send)

def drone_process_func(pipe_recv, glove_event):
    '''Process function to run the drone controller (pinned to core 1)'''
    print('Starting drone process...')
    os.sched_setaffinity(0, {1})
    glove_event.wait()
    tello = FALCON()
    tello.track_target(pipe_recv)

def main():
    '''Main entry point for the application'''

    # Configure the AP interface to be the correct IP address
    subprocess.run(['bash', Path(__file__).parent.parent.joinpath('scripts', 'config_ap.sh')], check=True)

    # Create the pipe and glove connection event
    pipe_recv, pipe_send = Pipe(duplex=False)
    glove_event = Event()

    # Block until the glove connects, then signal the event
    conn, sock = server.server_init(glove_event)

    # Create the server and drone processes
    server_proc = Process(target=server_process_func, args=(conn, sock, pipe_send), daemon=True)
    drone_proc = Process(target=drone_process_func, args=(pipe_recv, glove_event), daemon=True)

    # Start both processes
    server_proc.start()
    drone_proc.start()

    try:
        # Wait for both processes to complete
        server_proc.join()
        drone_proc.join()
    except KeyboardInterrupt:
        print('\nShutting down processes...')
        print('Processes terminated.')

if __name__ == '__main__':
    main()