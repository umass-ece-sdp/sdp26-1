import serial
import time

def send_command(ser, cmd):
    # Minor delay before sending the command so it isn't blasted instantly
    time.sleep(0.1)
    
    full_cmd = cmd + "\r\n"
    print(f">>> {cmd}")
    ser.write(full_cmd.encode('utf-8'))
    ser.flush()
    time.sleep(0.5)  # Give module time to process and respond
    
    # Read everything in the buffer instead of relying on in_waiting
    raw_response = ser.read_all()
    response = raw_response.decode('utf-8', errors='ignore').strip()
    
    print(f"<<<\n{response}")
    print("-" * 20)
    return response

def setup_module(port='/dev/ttyUSB0', baud=115200):
    try:
        # Disable DTR/RTS when opening to prevent unwanted resets
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 1
        ser.dtr = False
        ser.rts = False
        ser.open()
        
        print(f"Connected to {port} at {baud} baud.")
        
        # Wait for the module to 'boot up' if opening the port caused a minor power dip
        print("Waiting 1 second for module stability...")
        time.sleep(1.0)
        
        # Clear out any boot garbage from the buffer (e.g. +READY)
        if ser.in_waiting:
            print(f"Boot output: {ser.read_all().decode('utf-8', errors='ignore').strip()}")
        
        print("\nStarting AT Commands...")
        
        # 1. Test connection
        resp = send_command(ser, "AT")
        if "+OK" not in resp:
            print("WARNING: Module did not reply with +OK. Sequence may be frozen.")
        
        # 2. Check current mode and network settings
        send_command(ser, "AT+MODE?")
        send_command(ser, "AT+NETWORKID?")
        send_command(ser, "AT+ADDRESS?")
        
        # --- UNCOMMENT ONE OF THESE BLOCKS TO PROGRAM A MODULE ---
        
        # BLOCK 1: PROGRAM AS ANCHOR
        # print("\nProgramming as ANCHOR...")
        # send_command(ser, "AT+MODE=1")
        # send_command(ser, "AT+NETWORKID=REYAX123")
        # send_command(ser, "AT+ADDRESS=REYAX001")
        
        # BLOCK 2: PROGRAM AS TAG
        # print("\nProgramming as TAG...")
        # send_command(ser, "AT+MODE=0")
        # send_command(ser, "AT+NETWORKID=REYAX123")
        # send_command(ser, "AT+ADDRESS=REYAX002")
        
        ser.close()
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")

if __name__ == "__main__":
    setup_module()
