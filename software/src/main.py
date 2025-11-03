from software.lib import falcon, variables
from hardware.firmware import server
import multiprocessing


def server_process(shared_dict):
    '''Process function to run the server'''
    print('Starting server process...')
    server.run_server(shared_dict)


def drone_process(shared_dict):
    '''Process function to run the drone controller'''
    print('Starting drone process...')
    
    # Initialize shared variables for this process
    variables.init_shared(shared_dict)
    
    # Create a FALCON object to connect to the drone
    drone = falcon.FALCON()
    
    # Your drone control logic here
    # You can read instructions using: variables.get_instr()
    # Example:
    # while True:
    #     instruction = variables.get_instr()
    #     if instruction:
    #         print(f'Drone received instruction: {instruction}')
    #         # Process instruction...
    #     time.sleep(0.1)


if __name__ == '__main__':
    # Create a Manager to share variables between processes
    manager = multiprocessing.Manager()
    shared_dict = manager.dict()
    shared_dict['instruction'] = ''
    
    # Create the server and drone processes
    server_proc = multiprocessing.Process(target=server_process, args=(shared_dict,))
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