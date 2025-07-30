import numpy as np
import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial
import csv
import threading


from nml_hand_exo import HandExo, SerialComm


############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_names = ['FLX', 'EXT']

# max_flexion = 45   # Now calibrating during trials 
trial_count = 5  # Number of times the control structure loops 

shared_imu_data = 0
imu_lock = threading.Lock()

imu_update_enabled = threading.Event()
imu_update_enabled.set()


###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []
inlets = []
channel_labels = []


############### Connect to LSL Streams ###############
def resolve_lsl_streams():
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

# for name, inlet in inlets:
#     info = inlet.info()
#     n_channels = info.channel_count()
#     channel_labels.append((name, get_channel_labels(info, n_channels)))

# print("Channel labels:")
# for name, labels in channel_labels:
#     print(f"{name}: {labels}")



############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()


############# Reconfigure the exo just in case ###################

# Reboot motors
exo.reboot_motor(motorID)
exo.disable_motor(motorID)
time.sleep(1)

exo.enable_led(motorID)
exo.disable_led(motorID)
time.sleep(1)

# send_command(exo, "set_zero(motorID)")
time.sleep(1)

print("configured")

last_command = None  # Track last motor command to avoid repeats



############### Save recorded data to CSV: Thread Definition ############### 

def record_data(inlet, name):
    filename = f"jnd_lsl_record_{name}_{filename_timestamp}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            dir = get_latest_imu_angle()
            writer.writerow([timestamp] + sample + [dir])
            
            # writer.writerow([timestamp] + sample + [current_trigger_value])

        print(f"{name} recording stopped. Data saved to {filename}")


############### Gradual Motion Functions ############### 

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

def gradual_motion(motorID, currentMotorAngle, desiredMotorAngle):
    with imu_lock:
        exo.enable_motor(motor_id=motorID)
    intermediateAngles = np.linspace(currentMotorAngle, desiredMotorAngle, 100)
    print(f"gradually moving to motor angle {desiredMotorAngle}")
    for interAngle in intermediateAngles:
        with imu_lock:
            exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)


def gradual_flexion_until_angle(motorID, desiredWristAngle):
    with imu_lock:
        exo.enable_motor(motor_id=motorID)
    step_angle = 0.3
    print(f"gradually moving to wrist angle {desiredWristAngle}")
    with imu_lock:
        currentMotorAngle = exo.get_motor_angle(motor_id = motorID)
    time.sleep(1.5)

    moving = True
    while moving:
        newAngle = currentMotorAngle + step_angle
        with imu_lock:
            exo.set_motor_angle(motor_id=motorID, angle=newAngle)
        time.sleep(0.01)
        currentWristAngle = get_latest_imu_angle() #exo.get_imu_heading()
        print(currentWristAngle)
        if (abs(currentWristAngle - desiredWristAngle) <= 0.5):         # consider proportional motor implementation
            moving = False
        else:
            currentMotorAngle = currentMotorAngle + step_angle

def gradual_extension_until_angle(motorID, desiredWristAngle):
    exo.enable_motor(motor_id=motorID)
    step_angle = 0.3
    print(f"gradually moving to wrist angle {desiredWristAngle}")
    with imu_lock:
        currentMotorAngle = exo.get_motor_angle(motor_id = motorID)
    time.sleep(1.5)

    moving = True
    while moving:
        newAngle = currentMotorAngle - step_angle
        with imu_lock:
            exo.set_motor_angle(motor_id=motorID, angle=newAngle)
        time.sleep(0.01)
        currentWristAngle = get_latest_imu_angle() #exo.get_imu_heading()
        print(currentWristAngle)
        if abs(currentWristAngle - desiredWristAngle) <= 0.5:         # consider proportional motor implementation
            moving = False
        else:
            currentMotorAngle = currentMotorAngle - step_angle
    


############### Manually Control Assistance: Thread Definition ############### 

def control_structure():


    '''Calibration Stage'''
    # wait = input("Place wrist in a neutral position. Hit any key to record motor angle")
    # zero_angle = exo.get_motor_angle(motor_id=motorID)

    # wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will be the point to which the exo flexes the wrist.\n Hit any key to record the motor angle.")
    # max_flexion_angle = exo.get_motor_angle(motor_id=motorID)

    # wait = input("Move patients wrist to be as extended as possible, whether the limit is physiological or mechanical. This will be the point to which the exo extends the wrist.\n Hit any key to record the motor angle.")
    # max_extension_angle = exo.get_motor_angle(motor_id=motorID)

    wait = input("Place wrist in extended position. Hit any key to record motor/wrist angle")
    with imu_lock:
        zero_motor_angle = exo.get_motor_angle(motor_id=motorID)
    print(f"rest motor angle: {zero_motor_angle}")
    zero_wrist_angle = get_latest_imu_angle() #exo.get_imu_heading()
    print(f"zeros wrist angle: {zero_wrist_angle}")

    wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will define the bounds of the JND task if the participant does not have +- 15 degrees of motion in wrist joint.\n Hit any key to record the motor angle.")
    with imu_lock:
        max_motor_angle = exo.get_motor_angle(motor_id=motorID)
    print(f"max motor angle: {max_motor_angle}")
    max_flexion_wrist_angle = get_latest_imu_angle() # exo.get_imu_heading()
    print(f"Max flexion wrist angle: {max_flexion_wrist_angle:.2f}")


    if max_flexion_wrist_angle > zero_wrist_angle:
        print("crossed 360 threshold")
        max_flexion_wrist_angle_difference = (360-max_flexion_wrist_angle) + zero_wrist_angle
    else:
        print("did not cross 360 threshold")
        max_flexion_wrist_angle_difference = abs(max_flexion_wrist_angle - zero_wrist_angle)
    
    training_angle_difference = max_flexion_wrist_angle_difference * 0.8


    



    '''Control for Passive Intervention'''
    wait = input(f"Prepare for full passive intervention training ({trial_count} repetitions). Start in neutral wrist poisition, and hit Enter when ready")




    for i in range(trial_count):

        # _, flexion_timestamp = inlets[0][1].pull_sample()
        # trigger_timestamps.append((f"flexion_{i}", flexion_timestamp))

        print("Flexing")
        with imu_lock:
            exo.enable_motor(motor_id=motorID)
        startWristAngle = get_latest_imu_angle() #exo.get_imu_heading()
        targetAngle = startWristAngle - training_angle_difference
        if targetAngle < 0:
            print("adjusted target angle for 360 cross")
            targetAngle = targetAngle + 360
        print(f"target ref angle {i}:", targetAngle)
        time.sleep(2)
        gradual_flexion_until_angle(motorID, targetAngle)
        time.sleep(2)


        # _, extension_timestamp = inlets[0][1].pull_sample()
        # trigger_timestamps.append((f"extension_{i}", extension_timestamp))

        gradual_extension_until_angle(motorID, startWristAngle)
        time.sleep(1)


        

    with open(f"trigger_timestamps_{filename_timestamp}.txt", 'w') as timeFile:
        timeFile.write(" ".join(map(str, trigger_timestamps)))



    stop_event.set()



############### Main Function: Glorified Thread Manager ############### 

if __name__ == "__main__":

    filename_timestamp = int(time.time())

    # resolve_lsl_streams()

    threads = []
    trigger_timestamps = []

    stop_event = threading.Event()
    imu_update_enabled.set()

    for name, inlet in inlets:
        t = threading.Thread(target=record_data, args=(inlet, name))
        t.start()
        threads.append(t)

    t_control = threading.Thread(target=control_structure)
    t_control.start()
    threads.append(t_control)
    
    t_imu = threading.Thread(target=imu_manager)
    t_imu.start()
    threads.append(t_imu)

    
    
    for t in threads:
        t.join()

    print("End of the line")



