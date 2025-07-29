import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time
import pandas as pd
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial
import csv
import threading


from nml_hand_exo.hand_exo import HandExo




############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_name = 'SAGA'     #TODO: vary stream name so that we don't pick up the wrong device       # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi

max_flexion = 45   # Will change based on participant's maximum range of motion 
trial_count = 5  # Number of times the control structure loops 


###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []


###############  Connect to LSL Stream ############### 

print("Resolving LSL stream...")
#streams = resolve_stream('name', stream_name)
streams = pylsl.resolve_streams()

inlet = None
for i, stream in enumerate(streams):
    if stream.name() == stream_name:
        print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
        inlet = StreamInlet(stream, max_buflen=60)
        break

info = inlet.info()
n_channels = info.channel_count()

print(f"Connected to stream: {stream_name}")
print(f"Detected {n_channels} channels.")

# Get channel labels
channel_labels = []
try:
    chns = info.desc().child("channels").child("channel")
    for _ in range(n_channels):
        label = chns.child_value("label")
        channel_labels.append(label if label else f"ch{_}")
        chns = chns.next_sibling()
except Exception:
    print("!!!!!!! No channel names found. Using default labels.")
    channel_labels = [f"ch{i}" for i in range(n_channels)]
 
print("Channel labels:", channel_labels)


############### Backup for commands not in helper class ###############

def send_command(exo, msg):
    """Send a command to the exoskeleton."""
    if exo:
        try:
            if not msg.endswith('\n'):
                msg = msg + '\n'
            exo.write(msg.encode('utf-8'))
            print(f"Command sent: {msg.strip()}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Error sending command: {e}")
    else:
        print("Exoskeleton not connected, command not sent.")



############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

try:
    exo = HandExo(port=PORT, baudrate=BAUD_RATE, verbose=True)
    print(f"Connected to Exo on {PORT}")
except Exception as e:
    print(f"ERROR!!!!!!!!! Could not connect to {PORT}: {e}")
    exo = None

############# Reconfigure the exo just in case ###################        
# Reboot motors
exo.reboot_motor(motorID)
exo.disable_motor(motorID)
time.sleep(1)

exo.enable_led(motorID)
exo.disable_led(motorID)
time.sleep(1)


time.sleep(1)

print("configured")

last_command = None  # Track last motor command to avoid repeats



############### Save recorded data to CSV: Thread Definition ############### 

def record_data(): 
    filename = f"lsl_record_{filename_timestamp}.csv"
    with open(filename, mode = 'w', newline='') as file:
        writer = csv.writer(file)
        print("Recording streamed data")

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            writer.writerow([timestamp] + sample)
        print("\n Recording stopped. Data saved!")


############### Save recorded data to CSV: Thread Definition ############### 

def gradual_motion(motorID, currentAngle, desiredAngle):

    intermediateAngles = np.linspace(currentAngle, desiredAngle, 100)
    print("gradually moving")
    for interAngle in intermediateAngles:
        exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)
    


############### Manually Control Assistance: Thread Definition ############### 

def control_structure():


    '''Calibration Stage'''
    
    wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will be the point to which the exo flexes the wrist.\m Hit any key to record the motor angle.")
    max_flexion = exo.get_motor_angle(motor_id=motorID)


    '''Control for Manual AAN'''
    for i in range(trial_count):
        wait = input("Now, place wrist in neutral, \"zero\" position, and prepare to flex wrist. Hit Enter when ready")
        zero_angle = exo.get_motor_angle(motor_id=motorID)

        _, manual_flex_timestamp = inlet.pull_sample()
        trigger_timestamps.append((f"manual_flex_{i}", manual_flex_timestamp))
        print("Flex without help")


        wait = input("Wait for sustained max, at least 2-3 seconds, then hit Enter")
        participant_max_angle = exo.get_motor_angle(motor_id=motorID)

        _, assisted_flex_timestamp = inlet.pull_sample()
        trigger_timestamps.append((f"assisted_flex_{i}", assisted_flex_timestamp))
        print("flexing with assistance")
        exo.enable_motor(motorID)
        gradual_motion(motorID, participant_max_angle, max_flexion)


        wait = input("Wait for sustained max, at least 2-3 seconds, then hit Enter to return to zero position.")
        _, end_flex_timestamp = inlet.pull_sample()
        trigger_timestamps.append((f"end_flex_{i}", end_flex_timestamp))
        print("Completely relax wrist, and ensure that wrist returned to neutral position. ")
        gradual_motion(motorID, max_flexion, zero_angle)           #TODO: decide whether active assistance to return to 0 is necessary. Reduces use of extensors between trials, for better or worse
        exo.disable_motor(motorID)        
        
    with open(f"trigger_timestamps_{filename_timestamp}.txt", 'w') as timeFile:
        timeFile.write(" ".join(map(str, trigger_timestamps)))




    '''Gradual Motion Test'''

    # current = exo.get_motor_angle(motor_id=motorID)
    # goal = current + 40
    # exo.enable_motor(motor_id=motorID)
    # gradual_motion(motorID=motorID, desiredAngle=goal)
    # # gradual_motion(motorID=motorID, desiredAngle=current)
    # exo.disable_motor(motor_id=motorID)


    stop_event.set()



############### Main Function: Glorified Thread Manager ############### 

if __name__ == "__main__":
    
    filename_timestamp = int(time.time())

    trigger_timestamps = []

    t1 = threading.Thread(target=record_data)
    t2 = threading.Thread(target=control_structure)

    stop_event = threading.Event()

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    print("End of the line")



