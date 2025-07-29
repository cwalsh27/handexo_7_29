import numpy as np
import pylsl
from pylsl import StreamInlet
import time
import csv
import threading
import tkinter as tk

from nml_hand_exo import HandExo, SerialComm

############### Parameters ###############
stream_names = ['FLX', 'EXT']
sampling_rate = 2000  # Hz

PORT = 'COM6'
BAUD_RATE = 57600

############### Globals ###############
stop_event = threading.Event()
filename_timestamp = int(time.time())
inlets = []
channel_labels = []
current_trigger_value = 0  # 0 = LOW, 1 = HIGH

shared_imu_data = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
imu_lock = threading.Lock()

imu_update_enabled = threading.Event()
imu_update_enabled.set()

############### Connect to Exo on COM Port ###############
comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print('exo connected')

############### Connect to LSL Streams ###############
print("Resolving LSL streams...")
streams = pylsl.resolve_streams()

for name in stream_names:
    found = False
    for stream in streams:
        if stream.name() == name:
            print(f"Connected to LSL stream: {name}")
            inlet = StreamInlet(stream, max_buflen=60)
            inlets.append((name, inlet))
            found = True
            break
    if not found:
        print(f"Warning: LSL stream '{name}' not found.")

if len(inlets) == 0:
    raise RuntimeError("No required LSL streams found. Check stream names or network.")


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

for name, inlet in inlets:
    info = inlet.info()
    n_channels = info.channel_count()
    channel_labels.append((name, get_channel_labels(info, n_channels)))

print("Channel labels:")
for name, labels in channel_labels:
    print(f"{name}: {labels}")


############### Trigger Function (Virtual) ###############
def send_analog_trigger(value):
    global current_trigger_value
    current_trigger_value = value
    print(f"Simulated trigger: {value}")

############### IMU Management Thread ############### 
def imu_manager():
    while not stop_event.is_set():
        if imu_update_enabled.is_set():
            rpy = exo.get_imu_angles()
            if rpy is not None:
                with imu_lock:
                    shared_imu_data.update(rpy)
        time.sleep(0.05)

def get_latest_imu_angle():
    with imu_lock:
        return shared_imu_data.copy()

############### Data Recording Thread ###############
def record_data(inlet, name):
    filename = f"static_lsl_record_{name}_{filename_timestamp}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            if imu_update_enabled.is_set():
                rpy = get_latest_imu_angle()
                writer.writerow([timestamp] + sample + [rpy.values()] + [current_trigger_value])
            else:
                 writer.writerow([timestamp] + sample + [current_trigger_value])
            # writer.writerow([timestamp] + sample + [current_trigger_value])

        print(f"{name} recording stopped. Data saved to {filename}")



############### GUI and Protocol Logic ###############
def gui_thread():
    def protocol_sequence():
        # WAIT Phase
        status_label.config(text="WAIT", bg="lightblue")
        root.update()
        time.sleep(5)

        # START Phase (Trigger = 1)
        status_label.config(text="START", bg="lightgreen")
        send_analog_trigger(1)
        root.update()
        time.sleep(20)

        # END Phase (Trigger = 0)
        status_label.config(text="END", bg="tomato")
        send_analog_trigger(0)
        root.update()
        time.sleep(2)

        stop_event.set()
        root.quit()

    root = tk.Tk()
    root.title("Experiment Control")
    root.geometry("800x550")

    status_label = tk.Label(root, text="WAIT", font=("Helvetica", 50), width=150, height=40, bg="lightblue")
    status_label.pack(padx=20, pady=20)

    threading.Thread(target=protocol_sequence).start()
    root.mainloop()


############### Main Execution ###############
if __name__ == "__main__":
    # Start LSL recording threads
    threads = []
    for name, inlet in inlets:
        t = threading.Thread(target=record_data, args=(inlet, name))
        t.start()
        threads.append(t)

    # Start GUI thread
    t_gui = threading.Thread(target=gui_thread)
    t_gui.start()
    threads.append(t_gui)

    t_imu = threading.Thread(target=imu_manager)
    t_imu.start()
    threads.append(t_imu)

    # Wait for all threads to finish
    for t in threads:
        t.join()

    print("Experiment completed.")
