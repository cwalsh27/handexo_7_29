import numpy as np
import pylsl
from pylsl import StreamInlet  # , resolve_stream
import time
from scipy.signal import butter, lfilter, lfilter_zi, iirnotch, filtfilt
import serial
import csv
import threading
import random


from nml_hand_exo import HandExo, SerialComm


############### Change PARAMETERS here ############### 

sampling_rate = 2000  # Hz
PORT = 'COM6'
BAUD_RATE = 57600
motorID = 0

stream_names = ['FLX', 'EXT']     #TODO: vary stream name so that we don't pick up the wrong device       # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi

trial_count = 30      

shared_imu_data = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
imu_lock = threading.Lock()

imu_update_enabled = threading.Event()
imu_update_enabled.set()

###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []
inlets = []
channel_labels = []


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




############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

# try:
comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print(f"Connected to Exo on {PORT}")
# except Exception as e:
#     print(f"ERROR!!!!!!!!! Could not connect to {PORT}: {e}")
#     exo = None

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

def record_data(inlet, name):
    filename = f"static_lsl_record_{name}_{filename_timestamp}.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        while not stop_event.is_set():
            sample, timestamp = inlet.pull_sample()
            if imu_update_enabled.is_set():
                rpy = get_latest_imu_angle()
                writer.writerow([timestamp] + sample + [rpy.values()])
            else:
                 writer.writerow([timestamp] + sample)
            # writer.writerow([timestamp] + sample + [current_trigger_value])

        print(f"{name} recording stopped. Data saved to {filename}")



############### Gradual Motion Functions ############### 

def get_latest_imu_angle():
        with imu_lock:
            # print(shared_imu_data)
            return shared_imu_data.copy()

def gradual_motion(motorID, currentMotorAngle, desiredMotorAngle):
    exo.enable_motor(motor_id=motorID)
    intermediateAngles = np.linspace(currentMotorAngle, desiredMotorAngle, 100)
    print(f"gradually moving to motor angle {desiredMotorAngle}")
    for interAngle in intermediateAngles:
        exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)

def gradual_motion_until_angle(motorID, currentMotorAngle, desiredWristAngle):
    exo.enable_motor(motor_id=motorID)
    step_angle = 0.5
    print(f"gradually moving to wrist angle {desiredWristAngle}")

    moving = True
    while moving:
        newAngle = currentMotorAngle + step_angle
        exo.set_motor_angle(motor_id=motorID, angle=newAngle)
        time.sleep(0.05)
        currentWristAngle = get_latest_imu_angle()["yaw"]
        if currentWristAngle >= desiredWristAngle:         # consider proportional motor implementation
            moving = False
        else:
            currentMotorAngle = currentMotorAngle + step_angle
    


############### Manually Control Assistance: Thread Definition ###############

def control_structure():


    '''Initial Calibration Stage''' 
    wait = input("Place wrist in extended position. Hit any key to record motor/wrist angle")
    imu_update_enabled.clear()
    time.sleep(0.1)
    zero_motor_angle = exo.get_motor_angle(motor_id=motorID)
    imu_update_enabled.set()
    zero_wrist_angle = get_latest_imu_angle()["yaw"]
    print(zero_wrist_angle)

    wait = input("Move patients wrist to be as flexed as possible, whether the limit is physiological or mechanical. This will define the bounds of the JND task if the participant does not have +- 15 degrees of motion in wrist joint.\n Hit any key to record the motor angle.")
    # max_flexion_motor_angle = exo.get_motor_angle(motor_id=motorID)
    max_flexion_wrist_angle = get_latest_imu_angle()["yaw"]
    print(max_flexion_wrist_angle)

    # wait = input("Move patients wrist to be as extended as possible, whether the limit is physiological or mechanical. This will define the bounds of the JND task if the participant does not have +- 15 degrees of motion in wrist joint.\n Hit any key to record the motor angle.")
    # # max_extension_motor_angle = exo.get_motor_angle(motor_id=motorID)
    # max_extension_wrist_angle = get_latest_imu_angle()["yaw"]
    # print(max_extension_wrist_angle)

    # if abs(zero_wrist_angle - max_flexion_wrist_angle) > 15.0:          
    #     max_flexion_wrist_angle = zero_wrist_angle + 15.0

    # if abs(zero_wrist_angle - max_extension_wrist_angle) > 15.0:
    #     max_extension_wrist_angle = zero_wrist_angle - 15.0

    max_flexion_wrist_angle_difference = abs(zero_wrist_angle - max_flexion_wrist_angle)
    # max_extension_wrist_angle_difference = abs(zero_wrist_angle - max_extension_wrist_angle)



    '''Test Angle Initiation'''
    relative_flex_angles = [0.25] * 10 + [0.5] * 10 + [0.75] * 10  #rename based on direction of IMU movement
    relative_flex_angles = np.array(relative_flex_angles)
    print("relative flex: ", relative_flex_angles)
    flex_angle_differences = relative_flex_angles * max_flexion_wrist_angle_difference 
    # relative_extend_angles = [-0.25] * 5 + [-0.5] * 5 + [-0.75] * 5 
    # relative_extend_angles = np.array(relative_extend_angles)
    # extend_angles = relative_extend_angles * max_extension_wrist_angle_difference + zero_wrist_angle
    print("flex angles:", flex_angle_differences)
    # print("extend angles:", extend_angles)
    # reference_angles = np.concatenate(flex_angles, extend_angles)

    test_resolutions = [-2, -1, 0, 1, 2] * 6
    test_resolutions = np.array(test_resolutions)
    
    print("test resolutions:", test_resolutions)

    
    permutation = np.random.permutation(len(flex_angle_differences))

    flex_angle_differences = flex_angle_differences[permutation]
    test_resolutions = test_resolutions[permutation]

    print("ref angle differences:", flex_angle_differences)
    print("test resolutions:", test_resolutions)


    
    '''Testing Stage'''
    wait = input(f"Prepare for JND testing ({trial_count} repetitions). Start in neutral wrist poisition, and hit Enter when ready")


    trials_correct = 0

    answers = {
            "+" : 0,    #larger test angle
            "-" : 0,    #smaller test angle 
            "=" : 0,    #identical test angle
        }

    
    for i, ref_angle in enumerate(flex_angle_differences):
        
        

        trial_type = ''
        if test_resolutions[i] > 0:   
            trial_type = '+'
        elif test_resolutions[i] < 0:
            trial_type = '-'
        else:
            trial_type = '='
       
        
        print(f"reference angle {i}: {ref_angle}")
        exo.enable_motor(motor_id=motorID)
        targetAngle = get_latest_imu_angle()["yaw"] + ref_angle
        gradual_motion_until_angle(motorID=motorID, currentMotorAngle=zero_motor_angle, desiredWristAngle=targetAngle)
        time.sleep(2)
        imu_update_enabled.clear()
        time.sleep(0.2)
        currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        imu_update_enabled.set()
        gradual_motion(motorID=motorID, currentMotorAngle=currentMotorAngle, desiredMotorAngle=zero_motor_angle)
        time.sleep(1)

        
        
        exo.enable_motor(motor_id=motorID)
        test_angle = get_latest_imu_angle()["yaw"] + ref_angle + test_resolutions[i]
        print(f"test angle {i}: {test_angle}")
        gradual_motion_until_angle(motorID=motorID, currentMotorAngle=zero_motor_angle, desiredWristAngle=test_angle)
        time.sleep(2)

        imu_update_enabled.clear()
        time.sleep(0.2)
        currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        imu_update_enabled.set()
        gradual_motion(motorID=motorID, currentMotorAngle=currentMotorAngle, desiredMotorAngle=zero_motor_angle)
        time.sleep(1)

        
        participant_answer = input(f"Was the second angle larger, smaller, or identical to the first? Hit '+' for larger, '-' for smaller' or '=' for identical. (Correct answer: {test_resolutions[i]})")   

        if participant_answer=='z':
            print("early out")
            break

        if participant_answer in ["+", "-", "="]:
            answers[participant_answer] += 1
            
            if participant_answer == trial_type:
                trials_correct += 1

    with open(f"jndGuesses_{filename_timestamp}.txt", 'w') as guessFile:
        guessFile.write(f'Correct Responses over {trial_count} trials: {trials_correct}\n\n')
        guessFile.write('Dictionary of Answers:\n')
        guessFile.write(str(answers))
        guessFile.write('Order of Trials:')
        guessFile.write(f'Reference Angles: {str(flex_angle_differences)}\n')
        guessFile.write(f'Test Resolutions: {str(test_resolutions)}')
        # guessFile.write(f'Timestamps: {str(trigger_timestamps)}') #currently no trigger timestamps
            

    stop_event.set()

def imu_manager():
    while not stop_event.is_set():
        if imu_update_enabled.is_set():
            rpy = exo.get_imu_angles()
            if rpy is not None:
                with imu_lock:
                    shared_imu_data.update(rpy)
        time.sleep(0.05)


############### Main Function: Glorified Thread Manager ###############

if __name__ == "__main__":

    filename_timestamp = int(time.time())

    # trigger_timestamps = [] TODO: confirm this is obsolete

    threads = []

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

    stop_event = threading.Event()


    for t in threads:
        t.join()

    print("End of the line")



