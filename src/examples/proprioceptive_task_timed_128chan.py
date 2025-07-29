import numpy as np

import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time

from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial
import csv
import threading
import math
import tkinter as tk


from nml_hand_exo import HandExo, SerialComm




############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_names = ['FLX', 'EXT']  

max_flexion = 45   # Will change based on participant's maximum range of motion 
trial_count = 5  # Number of times the control structure loops TODO: increase this number before running

fps = 60  # For dial animation
repetitions = 5

# shared_imu_data = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
# imu_lock = threading.Lock()

# imu_update_enabled = threading.Event()
# imu_update_enabled.set()

###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []
inlets = []
channel_labels = []


############### Connect to LSL Streams ###############
# print("Resolving LSL streams...")
# streams = pylsl.resolve_streams()

# for name in stream_names:
#     found = False
#     for stream in streams:
#         if stream.name() == name:
#             print(f"Connected to LSL stream: {name}")
#             inlet = StreamInlet(stream, max_buflen=60)
#             inlets.append((name, inlet))
#             found = True
#             break
#     if not found:
#         print(f"Warning: LSL stream '{name}' not found.")

# if len(inlets) == 0:
#     raise RuntimeError("No required LSL streams found. Check stream names or network.")


############### Extract Channel Labels ###############
def get_channel_labels(info, n_channels):
    labels = []
    try:
        chns = info.desc().child("channels").child("channel")
        for _ in range(n_channels):
            label = chns.child_value("label")
            labels.append(label if label else f"ch{_}")
            chns = chns.next_sibling()
    except:
        labels = [f"ch{i}" for i in range(n_channels)]
    return labels

# for name, inlet in inlets:
#     info = inlet.info()
#     n_channels = info.channel_count()
#     channel_labels.append((name, get_channel_labels(info, n_channels)))

# print("Channel labels:")
# for name, labels in channel_labels:
#     print(f"{name}: {labels}")

############### Trigger Function ###############
def send_analog_trigger(value):
    global current_trigger_value
    current_trigger_value = value
    print(f"Trigger = {value}")


############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print(f"Connected to Exo on {PORT}")

############# Reconfigure the exo just in case ###################        
# Reboot motors
exo.reboot_motor(motorID)
exo.disable_motor(motorID)
time.sleep(1)

exo.enable_led(motorID)
exo.disable_led(motorID)
time.sleep(1)


print("configured")

last_command = None  # Track last motor command to avoid repeats



############### Save recorded data to CSV: Thread Definition ############### 

# def record_data(inlet, name):
#     filename = f"static_lsl_record_{name}_{filename_timestamp}.csv"
#     with open(filename, mode='w', newline='') as file:
#         writer = csv.writer(file)

#         while not stop_event.is_set():
#             sample, timestamp = inlet.pull_sample()
#             dir = exo.get_imu_heading()
#             writer.writerow([timestamp] + sample + [dir])
            
#             # writer.writerow([timestamp] + sample + [current_trigger_value])

#         print(f"{name} recording stopped. Data saved to {filename}")


############### Save recorded data to CSV: Thread Definition ############### 

# def get_latest_imu_angle():
#         with imu_lock:
#             # print(shared_imu_data)
#             return shared_imu_data.copy()

def gradual_motion(motorID, currentAngle, desiredAngle):
    exo.enable_motor(motor_id=motorID)  
    intermediateAngles = np.linspace(currentAngle, desiredAngle, 100)
    print("gradually moving")
    for interAngle in intermediateAngles:
        exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)


############### GUI + Dial Logic ###############
def gui_thread():
    def animate_sweep():
        # total_frames = int(duration * fps)
        # angle_step = (end_deg - start_deg) / total_frames

        def update(frame):
            
            angle_deg = exo.get_imu_heading()
            angle_rad = math.radians(angle_deg)
            x = center + needle_length * math.cos(angle_rad)
            y = center - needle_length * math.sin(angle_rad)
            canvas.coords(needle, center, center, x, y)
            send_analog_trigger(trigger_val)
            root.after(int(1000 / fps), lambda: update(frame + 1))

        update(0)

    def start_repetitions():
        send_analog_trigger(0)  # ensure trigger = 0 before anything starts
        canvas.configure(bg="lightgreen")

        def repeat(n):
            if n == 0:
                send_analog_trigger(0)  # final 5 sec = 0
                canvas.configure(bg="tomato")
                canvas.create_text(center, center, text="END", font=("Helvetica", 48, "bold"), fill="white")
                root.update()
                root.after(5000, lambda: (stop_event.set(), root.quit()))
                return

            # 90 → 180 (extension) → trigger = 1
            animate_sweep(90, 180, 1, lambda:
                # 180 → 90 (return) → trigger = 0
                animate_sweep(180, 90, 0, lambda:
                    repeat(n - 1)))

        repeat(repetitions)



    # GUI setup
    global root, canvas, needle, center, needle_length
    root = tk.Tk()
    root.title("Experiment Control")
    root.geometry("800x550")
    canvas_size = 1000
    center = canvas_size // 2
    needle_length = 250

    canvas = tk.Canvas(root, width=canvas_size, height=canvas_size, bg="lightblue")
    canvas.pack()

    needle = canvas.create_line(center, center, center, center - needle_length, width=6, fill="black")

    canvas.create_text(center, center - 300, text="Neutral", font=("Helvetica", 16, "bold"), fill="black")
    canvas.create_text(center - needle_length - 100, center, text="Full Extension", font=("Helvetica", 16, "bold"), fill="black")

    root.after(5000, start_repetitions)  # start after 5s
    root.mainloop()
    


############### Manually Control Assistance: Thread Definition ############### 

def control_structure():


    '''Calibration Stage'''
    
    wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will be the point to which the exo flexes the wrist.\m Hit any key to record the motor angle.")
    max_flexion = exo.get_motor_angle(motor_id=motorID)


    '''Control for Manual AAN'''
    for i in range(trial_count):
        wait = input("Now, place wrist in neutral, \"zero\" position, and prepare to flex wrist. Hit Enter when ready")
        zero_angle = exo.get_motor_angle(motor_id=motorID)

        # _, manual_flex_timestamp = inlet.pull_sample() TODO: uncomment the trigger timestampss
        # trigger_timestamps.append((f"manual_flex_{i}", manual_flex_timestamp))
        print("Flex without help")


        time.sleep(2.5)
        if exo.get_motor_angle(motor_id=motorID) < max_flexion:
            participant_max_angle = exo.get_motor_angle(motor_id=motorID)

            # _, assisted_flex_timestamp = inlet.pull_sample()
            # trigger_timestamps.append((f"assisted_flex_{i}", assisted_flex_timestamp))
            print("flexing with assistance")
            exo.enable_motor(motorID)
            gradual_motion(motorID, participant_max_angle, max_flexion)


        time.sleep(2)
        # _, end_flex_timestamp = inlet.pull_sample()
        # trigger_timestamps.append((f"end_flex_{i}", end_flex_timestamp))
        print("Completely relax wrist, and ensure that wrist returned to neutral position. ")
        gradual_motion(motorID, max_flexion, zero_angle)           #TODO: decide whether active assistance to return to 0 is necessary. Reduces use of extensors between trials, for better or worse
        exo.disable_motor(motorID)        
        
        
    with open(f"trigger_timestamps_{filename_timestamp}.txt", 'w') as timeFile:
        timeFile.write(" ".join(map(str, trigger_timestamps)))




    stop_event.set()


# def imu_manager():
#     while not stop_event.is_set():
#         if imu_update_enabled.is_set():
#             rpy = exo.get_imu_angles()
#             if rpy is not None:
#                 with imu_lock:
#                     shared_imu_data.update(rpy)
#         time.sleep(0.05)


############### Main Function: Glorified Thread Manager ############### 

if __name__ == "__main__":
    
    filename_timestamp = int(time.time())

    trigger_timestamps = []
    threads = []

    # for name, inlet in inlets: # TODO: add back data recording and everything to do with it
    #     t = threading.Thread(target=record_data, args=(inlet, name))
    #     t.start()
    #     threads.append(t)

    t_gui = threading.Thread(target=gui_thread)
    t_gui.start()
    threads.append(t_gui)
    
    t_control = threading.Thread(target=control_structure)
    t_control.start()
    threads.append(t_control)

    # t_imu = threading.Thread(target=imu_manager)
    # t_imu.start()
    # threads.append(t_imu)


    stop_event = threading.Event()

    for t in threads:
        t.join()

    print("End of the line")



