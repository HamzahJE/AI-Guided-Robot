import time
import sys
import os
import serial
import threading
from dotenv import load_dotenv

project_root = os.path.dirname(os.path.abspath(__file__))
load_dotenv(os.path.join(project_root, '.env'))

# Make sure your capture_image function can return an image object or save to a known filepath rapidly
from modules.cam import capture_image 
from modules.openai_vision import get_driving_command

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Global variable for the async camera thread
latest_image_ready = False

def open_serial(port=SERIAL_PORT, baud=BAUD_RATE):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print(f"[serial] Connected to Arduino on {port} @ {baud}")
    return ser

def read_front_distance(ser):
    """Read any pending sensor telemetry (D:<cm>) from Arduino and return latest distance."""
    dist = -1
    while ser.in_waiting:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line.startswith('D:'):
                dist = int(line[2:])
        except (ValueError, UnicodeDecodeError):
            pass
    return dist

def send_goal(ser, goal):
    ser.reset_input_buffer()
    ser.write(goal.encode())
    time.sleep(0.05)
    ack = ser.read(1).decode(errors='ignore')
    if ack == 'A':
        print(f"[arduino] ACK — goal '{goal}' applied")

def camera_worker(stop_event):
    """Background thread continuously capturing the latest frame."""
    global latest_image_ready
    print("[cam_thread] Starting async camera capture...")
    while not stop_event.is_set():
        try:
            # Modify this if your function returns an image object instead of saving to disk
            capture_image() 
            latest_image_ready = True
            time.sleep(0.1) # Prevent CPU hogging
        except Exception as e:
            print(f"[cam_thread] ERROR: {e}")
            time.sleep(1)

def main():
    ser = open_serial()

    stop_event = threading.Event()
    
    # Start Asynchronous Camera Thread
    cam_thread = threading.Thread(target=camera_worker, args=(stop_event,), daemon=True)
    cam_thread.start()

    print("[main] Starting AI driving loop. Arduino handles reflexes, Pi handles strategy.")
    send_goal(ser, 'S')

    last_goal = 'S'
    front_dist = -1

    try:
        while True:
            global latest_image_ready

            # Read latest sensor telemetry from Arduino
            d = read_front_distance(ser)
            if d >= 0:
                front_dist = d

            # Wait for the camera thread to have a fresh frame ready
            if not latest_image_ready:
                time.sleep(0.05)
                continue

            latest_image_ready = False # Consume the frame

            print(f"[llm] Processing frame. Last action was '{last_goal}', front dist: {front_dist}cm")

            # Context-Aware Prompting (Short-Term Memory)
            dist_info = f"Ultrasonic sensor: front obstacle at {front_dist}cm. " if front_dist >= 0 else ""
            dynamic_prompt = (
                f"Last move: {last_goal}. {dist_info}"
                "Look at the ground directly in front of you. "
                "If the forward path has ANY obstacle within 50cm, you MUST turn (L or R) — do NOT say F. "
                "Pick a turn direction and commit to it — do not alternate between L and R. "
                "Otherwise, steer towards the widest clear gap. Output exactly 1 letter."
            )

            try:
                # Ensure your get_driving_command function accepts the prompt parameter!
                goal = get_driving_command(dynamic_prompt)
            except Exception as e:
                print(f"[llm] ERROR: {e}")
                send_goal(ser, 'S')
                continue

            if goal in ('F', 'B', 'L', 'R', 'S'):
                print(f"[llm] New goal: {goal}")
                send_goal(ser, goal)
                if goal != 'S': 
                    last_goal = goal # Remember what we did for the next loop
            else:
                send_goal(ser, 'S')

    except KeyboardInterrupt:
        print("\n[main] Stopping...")
        send_goal(ser, 'S')
        stop_event.set()
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    main()