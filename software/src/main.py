from software.lib import falcon, variables
from hardware.firmware import server
import multiprocessing, socket


def server_process(shared_dict: dict, conn: socket.socket, sock: socket.socket):
    '''Process function to run the server'''
    print('Starting server process...')
    server.run_server(shared_dict, conn, sock)

def drone_process(shared_dict):
    '''Process function to run the drone controller'''
    print('Starting drone process...')
    
    # Initialize shared variables for this process
    variables.init_shared(shared_dict)
    
    # Create a FALCON object to connect to the drone
    # drone = falcon.FALCON()

    prev = ''
    while True:
        current = variables.get_instr()
        if not current == prev:
            print(f'Instruction seen in drone process ----- {variables.get_instr()}')
        prev = current
    
    # Your drone control logic here
    # You can read instructions using: variables.get_instr()
    # Example:
    # while True:
    #     instruction = variables.get_instr()
    #     if instruction:
    #         print(f'Drone received instruction: {instruction}')
    #         # Process instruction...
    #     time.sleep(0.1)

def main():
    '''Main entry point for the application'''
    # Create a Manager to share variables between processes
    manager = multiprocessing.Manager()
    shared_dict = manager.dict()
    shared_dict['instruction'] = ''

    # Start the server and connect to glove before starting process
    conn, sock = server.server_init()
    
    # Create the server and drone processes
    server_proc = multiprocessing.Process(target=server_process, args=(shared_dict, conn, sock,))
    drone_proc = multiprocessing.Process(target=drone_process, args=(shared_dict,))
    
    # Start both processes
    server_proc.start()
    drone_proc.start()
    
    try:
        # Wait for both processes to complete
        server_proc.join()
        drone_proc.join()
    except KeyboardInterrupt:
        print('\nShutting down processes...')
        server_proc.terminate()
        drone_proc.terminate()
        server_proc.join()
        drone_proc.join()
        print('Processes terminated.')

if __name__ == '__main__':
    main()