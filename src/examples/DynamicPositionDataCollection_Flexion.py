import numpy as np
import pylsl
from pylsl import StreamInlet
import time
import csv
import threading
import tkinter as tk
import math

from nml_hand_exo import HandExo, SerialComm

############### Parameters ###############
PORT = 'COM5'
BAUD_RATE = 57600

stream_names = ['FLX', 'EXT']
sampling_rate = 2000  # Hz
speed = "slow"  # Options: "slow", "medium", "fast"
repetitions = 5

speed_map = {
    "slow": 5.0,
    "medium": 2.5,
    "fast": 1.0
}
duration = speed_map[speed]
start_angle = 90
end_angle = 180
fps = 60  # For dial animation

############### Globals ###############
stop_event = threading.Event()
filename_timestamp = int(time.time())
inlets = []
channel_labels = []
current_trigger_value = 0  # 0 = LOW, 1 = HIGH

shared_imu_data = 0
imu_lock = threading.Lock()

imu_update_enabled = threading.Event()
imu_update_enabled.set()


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

############### Connect to Exo on COM Port ###############
comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print('exo connected')

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


############### Trigger Function ###############
def send_analog_trigger(value):
    global current_trigger_value
    current_trigger_value = value
    print(f"Trigger = {value}")

############### IMU Management Thread ############### 
def imu_manager():
    global shared_imu_data
    while not stop_event.is_set():
        if imu_update_enabled.is_set():
            with imu_lock:
                dir = exo.get_imu_heading()
                # print("dir:", dir)
                if dir is not None:
                    shared_imu_data = dir
        time.sleep(0.01)

def get_latest_imu_angle():
        with imu_lock:
            # print(shared_imu_data)
            return shared_imu_data

############### Data Recording Thread ###############
def record_data(inlet, name):
    filename = f"dynamic_lsl_record_{name}_{filename_timestamp}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            dir = get_latest_imu_angle()
            writer.writerow([timestamp] + sample + [dir])
            # if imu_update_enabled.is_set():
            #     rpy = get_latest_imu_angle()
            #     writer.writerow([timestamp] + sample + [rpy.values()] + [current_trigger_value])
            # else:
            #      writer.writerow([timestamp] + sample + [current_trigger_value])
            # writer.writerow([timestamp] + sample + [current_trigger_value])
        print(f"{name} recording stopped. Data saved to {filename}")


############### GUI + Dial Logic ###############
def gui_thread():
    def animate_sweep(start_deg, end_deg, trigger_val, callback=None):
        total_frames = int(duration * fps)
        angle_step = (end_deg - start_deg) / total_frames

        def update(frame):
            if frame > total_frames:
                if callback:
                    callback()
                return
            angle_deg = start_deg + frame * angle_step
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


############### Main Execution ###############
if __name__ == "__main__":
    threads = []
    for name, inlet in inlets:
        t = threading.Thread(target=record_data, args=(inlet, name))
        t.start()
        threads.append(t)

    t_gui = threading.Thread(target=gui_thread)
    t_gui.start()
    threads.append(t_gui)

    t_imu = threading.Thread(target=imu_manager)
    t_imu.start()
    threads.append(t_imu)

    for t in threads:
        t.join()

    print("Experiment completed.")
