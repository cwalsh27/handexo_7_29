import numpy as np
import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial
import csv
import threading


#from nml_hand_exo import HandExo, SerialComm


############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_name_1 = 'FLX'
stream_name_2 = 'EXT'     #TODO: vary stream name so that we don't pick up the wrong device       # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi

# max_flexion = 45   # Now calibrating during trials 
trial_count = 5  # Number of times the control structure loops 


###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []


###############  Connect to LSL Stream ############### 

print("Resolving LSL stream...")
#streams = resolve_stream('name', stream_name)
streams = pylsl.resolve_streams()

inlet1 = None
inlet2 = None
j = 0
for i, stream in enumerate(streams):
    print('STREAM NAME: ', stream.name())
    if stream.name() == stream_name_1:
        print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
        inlet1 = StreamInlet(stream, max_buflen=60)
        j += 1

    if stream.name() == stream_name_2:
        print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
        inlet2 = StreamInlet(stream, max_buflen=60)
        j += 1

    if j > 2:
        break

info1 = inlet1.info()
info2 = inlet2.info()
n_channels1 = info1.channel_count()
n_channels2 = info2.channel_count()

print(f"Connected to stream(s): {stream_name_1, stream_name_2}")
print(f"Detected {n_channels1, n_channels2} channels.")

# Get channel labels
channel_labels1 = []
channel_labels2 = []
try:
    chns = info1.desc().child("channels").child("channel")
    for _ in range(n_channels1):
        label = chns.child_value("label")
        channel_labels1.append(label if label else f"ch{_}")
        chns = chns.next_sibling()
except Exception:
    print("!!!!!!! No channel names found. Using default labels.")
    channel_labels = [f"ch{i}" for i in range(n_channels1)]

try:
    chns = info2.desc().child("channels").child("channel")
    for _ in range(n_channels2):
        label = chns.child_value("label")
        channel_labels2.append(label if label else f"ch{_}")
        chns = chns.next_sibling()
except Exception:
    print("!!!!!!! No channel names found. Using default labels.")
    channel_labels = [f"ch{i}" for i in range(n_channels2)]
 
print("Channel labels:", channel_labels1)
print("Channel labels:", channel_labels2)






############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

# comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
# exo = HandExo(comm, verbose=False)
# exo.connect()


############# Reconfigure the exo just in case ###################

# Reboot motors
# exo.reboot_motor(motorID)
# exo.disable_motor(motorID)
# time.sleep(1)

# exo.enable_led(motorID)
# exo.disable_led(motorID)
time.sleep(1)

# send_command(exo, "set_zero(motorID)")
time.sleep(1)

print("configured")

last_command = None  # Track last motor command to avoid repeats




############### Save recorded data to CSV: Thread Definition ############### 

def record_data1(): 
    filename = f"lsl_record_chan1_{filename_timestamp}.csv"
    with open(filename, mode = 'w', newline='') as file:
        writer = csv.writer(file)
        print("Recording streamed data chan1")

        while not stop_event.is_set():
            sample, timestamp = inlet1.pull_sample()
            writer.writerow([timestamp] + sample)
        print("\n Recording stopped. chan1 Data saved!")

def record_data2():
    filename = f"lsl_record_chan2_{filename_timestamp}.csv"
    with open(filename, mode = 'w', newline='') as file:
        writer = csv.writer(file)
        print("Recording streamed data chan2")

        while not stop_event.is_set():
            sample, timestamp = inlet2.pull_sample()
            writer.writerow([timestamp] + sample)
        print("\n Recording stopped. chan2 Data saved!")


############### Save recorded data to CSV: Thread Definition ############### 

# def gradual_motion(motorID, currentAngle, desiredAngle):

#     intermediateAngles = np.linspace(currentAngle, desiredAngle, 100)
#     print("gradually moving")
#     for interAngle in intermediateAngles:
#         exo.set_motor_angle(motor_id=motorID, angle=interAngle)
#         time.sleep(0.01)
    


############### Manually Control Assistance: Thread Definition ############### 

# def control_structure():


#     '''Calibration Stage'''
#     wait = input("Place wrist in a neutral position. Hit any key to record motor angle")
#     zero_angle = exo.get_motor_angle(motor_id=motorID)

#     wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will be the point to which the exo flexes the wrist.\m Hit any key to record the motor angle.")
#     max_flexion_angle = exo.get_motor_angle(motor_id=motorID)

#     wait = input("Move patients wrist to be as extended as possible, whether the limit is physiological or mechanical. This will be the point to which the exo extends the wrist.\m Hit any key to record the motor angle.")
#     max_extension_angle = exo.get_motor_angle(motor_id=motorID)



'''Control for Passive Intervention'''
    # wait = input(f"Prepare for full passive intervention training ({trial_count} repetitions). Start in neutral wrist poisition, and hit Enter when ready")

    # _, extension_timestamp = inlet.pull_sample()
    # trigger_timestamps.append(("InitialExtension", extension_timestamp))

    # print("Extending")
    # exo.enable_motor(motor_id=motorID)
    # gradual_motion(motorID, zero_angle, max_extension_angle)
    # time.sleep(1)

    # for i in range(trial_count):
    #     _, flexion_timestamp = inlet.pull_sample()
    #     trigger_timestamps.append((f"flexion_{i}", flexion_timestamp))

    #     print("Flexing")
    #     exo.enable_motor(motorID)
    #     gradual_motion(motorID, max_extension_angle, max_flexion_angle)
    #     time.sleep(1)

        
    #     _, extension_timestamp = inlet.pull_sample()
    #     trigger_timestamps.append((f"extension_{i}", extension_timestamp))

    #     print("Extending")
    #     exo.enable_motor(motorID)
    #     gradual_motion(motorID, max_flexion_angle, max_extension_angle)
    #     time.sleep(1)

        

    # with open(f"trigger_timestamps_{filename_timestamp}.txt", 'w') as timeFile:
    #     timeFile.write(" ".join(map(str, trigger_timestamps)))



    # stop_event.set()



############### Main Function: Glorified Thread Manager ############### 

if __name__ == "__main__":

    filename_timestamp = int(time.time())

    trigger_timestamps = []

    t1 = threading.Thread(target=record_data1)
    t2 = threading.Thread(target=record_data2)
    # t3 = threading.Thread(target=control_structure)

    stop_event = threading.Event()

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    wait = input("press a key to end")

    stop_event.set()

    print("End of the line")



