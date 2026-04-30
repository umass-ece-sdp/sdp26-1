import cv2
import numpy as np
import threading
import sys
import os
import time

# Add the project root to the python path so absolute imports like 'software...' work
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from software.src.glove_client import glove_client_init, run_glove_client
from software.lib import variables

# Per-finger threshold (volts). Voltage RISES as the finger bends down (pulled).
#   voltage >= threshold -> finger PULLED (1, Green)
#   voltage <  threshold -> finger NOT PULLED (0, Red)
FINGER_UP_THRESHOLD = [
    0.17,   # [0] pinky  (resting ~0.15, pulled ~0.19)
    0.22,   # [1] ring   (resting ~0.20, pulled ~0.24)
    0.19,   # [2] middle (resting ~0.17, pulled ~0.21)
    0.22,   # [3] index  (resting ~0.20, pulled ~0.24)
]

# Order of fingers left-to-right on left hand: Index, Middle, Ring, Pinky
FINGER_ORDER = [3, 2, 1, 0]

GESTURE_MAP = {
    '1000': 'arc_left',
    '1100': 'arc_right',
    '0000': 'quit',
    '0100': 'flip',
    '1001': 'up',
    '0011': 'down',
    '1010': 'further',
    '0110': 'closer',
}

def calibrate_glove_thresholds():
    global FINGER_UP_THRESHOLD
    import time
    print("\n" + "="*50)
    print("  GLOVE CALIBRATION")
    print("="*50)
    
    def get_fingers():
        instr = variables.read_instr()
        if 'fingers' in instr:
             return instr['fingers']
        return (0.0, 0.0, 0.0, 0.0)
    
    # Wait until we get a clear "released" state first (-1.0 on all fingers)
    if get_fingers()[0] >= -0.5:
        print("Please release the button on the glove to begin...")
        while get_fingers()[0] >= -0.5:
            time.sleep(0.05)

    print("Keep your hand flat (fingers unbent). Press the button on the glove to record.")
    while get_fingers()[0] < -0.5:
        time.sleep(0.05)
    
    # Short delay to let the reading stabilize after the button press
    time.sleep(0.1)
    straight_voltages = get_fingers()
    print(f"Recorded straight: {straight_voltages}")

    # Wait for the user to release the button
    while get_fingers()[0] >= -0.5:
        time.sleep(0.05)

    print("\nBend all your fingers. Press the button on the glove to record.")
    while get_fingers()[0] < -0.5:
        time.sleep(0.05)
        
    # Short delay to stabilize
    time.sleep(0.1)
    bent_voltages = get_fingers()
    print(f"Recorded bent: {bent_voltages}")
    
    # Wait for release to prevent accidental triggering after calibration
    while get_fingers()[0] >= -0.5:
        time.sleep(0.05)

    for i in range(4):
        FINGER_UP_THRESHOLD[i] = (straight_voltages[i] + bent_voltages[i]) / 2.0
    
    print("\nNew Thresholds (FINGER_UP_THRESHOLD):")
    print(f"  [0] pinky:  {FINGER_UP_THRESHOLD[0]:.3f}")
    print(f"  [1] ring:   {FINGER_UP_THRESHOLD[1]:.3f}")
    print(f"  [2] middle: {FINGER_UP_THRESHOLD[2]:.3f}")
    print(f"  [3] index:  {FINGER_UP_THRESHOLD[3]:.3f}")
    print("="*50 + "\n")

def main():
    # Initialize the glove connection using the existing client logic
    sock = glove_client_init()
    if not sock:
        print("Failed to initialize glove socket. Exiting.")
        return
        
    # Start the existing glove client logic in a background daemon thread.
    # It continuously reads and writes data to shared `variables.py`.
    recv_thread = threading.Thread(
        target=run_glove_client, 
        args=(sock, False), 
        daemon=True
    )
    recv_thread.start()

    window_name = "Glove Demonstration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)
    
    finger_names = ["Index", "Middle", "Ring", "Pinky"]
    
    print("Starting display. Press 'ESC' in the window to quit.")
    
    # Run calibration before starting visualization
    time.sleep(1) # Let the background thread grab some values
    calibrate_glove_thresholds()
    
    # Store the last valid known finger state (1s and 0s) and voltages
    # We initialize to all 0s (not pulled/Red)
    last_valid_bits = [0, 0, 0, 0]
    last_valid_volts = [0.0, 0.0, 0.0, 0.0]
    
    try:
        while True:
            # Create a blank canvas (dark gray)
            img = np.ones((480, 640, 3), dtype=np.uint8) * 30
            
            # Read the thread-safe instruction from variables.py
            instr = variables.read_instr()
            
            if 'fingers' in instr:
                fingers = instr['fingers']
            else:
                # Default empty state before any data arrives
                fingers = (0.0, 0.0, 0.0, 0.0)
            
            # The C++ firmware returns -1.0 for all fingers when the button is RELEASED.
            # We can use that to accurately toggle our button indicator.
            button_pressed = (fingers[0] >= 0.0)
            
            # Determine pattern
            # If the button isn't pressed, the firmware hides finger data (-1.0).
            # We freeze the display to the last known state so the hand doesn't flicker.
            if button_pressed:
                bits = []
                for idx in FINGER_ORDER:
                    # Voltage RISES when pulled. 
                    # pulled (>= threshold) = 1 (Green), not pulled (< threshold) = 0 (Red)
                    is_pulled = 1 if fingers[idx] >= FINGER_UP_THRESHOLD[idx] else 0
                    bits.append(is_pulled)
                last_valid_bits = bits
                last_valid_volts = fingers  # Snapshot real voltages
            else:
                bits = last_valid_bits
                
            # Draw fingers
            start_x = 100
            for i, bit in enumerate(bits):
                # We need the true packet index corresponding to this on-screen array
                idx = FINGER_ORDER[i]
                
                # Using a rounded rectangle design for fingers
                rect_x, rect_y = start_x + i * 110, 150
                rect_w, rect_h = 70, 160
                
                # Colors: Green if pulled (1), Red if resting/not pulled (0)
                color = (0, 200, 0) if bit == 1 else (0, 0, 200)
                
                # Fill the rectangle
                cv2.rectangle(img, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), color, -1)
                # Border
                cv2.rectangle(img, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (255, 255, 255), 2)
                
                # Label
                cv2.putText(img, finger_names[i], (rect_x, rect_y + rect_h + 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
                            
                # Raw voltage reading (to help diagnose thresholds)
                volts = last_valid_volts[idx]
                cv2.putText(img, f"{volts:.2f}V", (rect_x, rect_y + rect_h + 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

            # Draw the permutation string at the top
            pattern_str = ''.join(map(str, bits))
            cv2.putText(img, f"Hand Permutation: {pattern_str}", (100, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)
            
            # Map bits for command lookup (glove-and-flight expects 1 for up, 0 for down)
            mapped_str = ''.join(['0' if b == 1 else '1' for b in bits])
            cmd = GESTURE_MAP.get(mapped_str, 'None')
            
            # Display current command
            cv2.putText(img, f"Command: {cmd}", (100, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
            
            # Draw an indicator for the "Event Listener" Button
            # The button_pressed variable was set above based on the firmware sending -1.0
            btn_color = (0, 255, 255) if button_pressed else (80, 80, 80)
            btn_radius = 20
            btn_center = (320 - 70, 410)
            cv2.circle(img, btn_center, btn_radius, btn_color, -1)
            cv2.circle(img, btn_center, btn_radius, (255, 255, 255), 2)
            
            btn_text = "Button: PRESSED" if button_pressed else "Button: RELEASED"
            cv2.putText(img, btn_text, (btn_center[0] + 35, btn_center[1] + 8), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, btn_color, 2, cv2.LINE_AA)
            
            # Show the image
            cv2.imshow(window_name, img)
            
            # Polling delay + listen for Escape Key
            if cv2.waitKey(30) & 0xFF == 27: # ESC
                break
                
    except KeyboardInterrupt:
        print("Keyboard interrupt received.")
    finally:
        # sock.close() happens automatically if the thread is killed
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
