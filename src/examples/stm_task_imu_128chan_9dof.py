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
PORT = 'COM5'
BAUD_RATE = 57600
motorID = 0

stream_name_1 = 'FLX'
stream_name_2 = 'EXT'     #TODO: vary stream name so that we don't pick up the wrong device       # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi

encoding_round_count = 1     # 3 exposures per memory angle  (chiyohara 2020)
trial_count = 60      # Number of trials (MUST BE A MULTIPLE OF 6) 

# shared_imu_data = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
# imu_lock = threading.Lock()
# shared_heading = 0

# imu_update_enabled = threading.Event()
# imu_update_enabled.set()

###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []


###############  Connect to LSL Stream ############### # won't need the emg data during the stm task

# print("Resolving LSL stream...")
# #streams = resolve_stream('name', stream_name)
# streams = pylsl.resolve_streams()

# inlet1 = None
# inlet2 = None
# j = 0
# for i, stream in enumerate(streams):
#     print('STREAM NAME: ', stream.name())
#     if stream.name() == stream_name_1:
#         print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
#         inlet1 = StreamInlet(stream, max_buflen=60)
#         j += 1

#     if stream.name() == stream_name_2:
#         print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
#         inlet2 = StreamInlet(stream, max_buflen=60)
#         j += 1

#     if j > 2:
#         break

# info1 = inlet1.info()
# info2 = inlet2.info()
# n_channels1 = info1.channel_count()
# n_channels2 = info2.channel_count()

# print(f"Connected to stream(s): {stream_name_1, stream_name_2}")
# print(f"Detected {n_channels1, n_channels2} channels.")

# # Get channel labels
# channel_labels1 = []
# channel_labels2 = []
# try:
#     chns = info1.desc().child("channels").child("channel")
#     for _ in range(n_channels1):
#         label = chns.child_value("label")
#         channel_labels1.append(label if label else f"ch{_}")
#         chns = chns.next_sibling()
# except Exception:
#     print("!!!!!!! No channel names found. Using default labels.")
#     channel_labels = [f"ch{i}" for i in range(n_channels1)]

# try:
#     chns = info2.desc().child("channels").child("channel")
#     for _ in range(n_channels2):
#         label = chns.child_value("label")
#         channel_labels2.append(label if label else f"ch{_}")
#         chns = chns.next_sibling()
# except Exception:
#     print("!!!!!!! No channel names found. Using default labels.")
#     channel_labels = [f"ch{i}" for i in range(n_channels2)]
 
# print("Channel labels:", channel_labels1)
# print("Channel labels:", channel_labels2)



############### Connect to Exo on COM Port ###############

# Change COM based on exo connection
# Change baudrate if using old dynamixel motor to 1000000

comm = SerialComm(port=PORT, baudrate=BAUD_RATE)
exo = HandExo(comm, verbose=False)
exo.connect()
print('exo connected')
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

# send_command(exo, "set_zero(motorID)")
time.sleep(1)

print("configured")

last_command = None  # Track last motor command to avoid repeats



############### Save recorded data to CSV: Thread Definition ############### 

# def record_data1(): 
#     filename = f"lsl_record_chan1_{filename_timestamp}.csv"
#     with open(filename, mode = 'w', newline='') as file:
#         writer = csv.writer(file)
#         print("Recording streamed data chan1")

#         while not stop_event.is_set():
#             sample, timestamp = inlet1.pull_sample()
#             if imu_update_enabled.is_set():
#                 rpy = exo.get_imu_angles()
#                 writer.writerow([timestamp] + sample + rpy)
#             else:
#                  writer.writerow([timestamp] + sample)
#         print("\n Recording stopped. chan1 Data saved!")

# def record_data2():
#     filename = f"lsl_record_chan2_{filename_timestamp}.csv"
#     with open(filename, mode = 'w', newline='') as file:
#         writer = csv.writer(file)
#         print("Recording streamed data chan2")

#         while not stop_event.is_set():
#             sample, timestamp = inlet2.pull_sample()
#             if imu_update_enabled.is_set():
#                 rpy = exo.get_imu_angles()
#                 writer.writerow([timestamp] + sample + rpy)
#             else:
#                  writer.writerow([timestamp] + sample)
#         print("\n Recording stopped. chan2 Data saved!")



############### IMU Management Thread ############### 
# def imu_manager():
#     while not stop_event.is_set():
#         heading = exo.get_imu_heading()
#         if heading != None:
#             shared_heading = heading
#         time.sleep(0.01)

# def get_latest_heading():
#     with imu_lock:
#         return shared_heading

############### Gradual Motion Functions ############### 

def gradual_motion(motorID, currentMotorAngle, desiredMotorAngle):
    exo.enable_motor(motor_id=motorID)
    intermediateAngles = np.linspace(currentMotorAngle, desiredMotorAngle, 100)
    print(f"gradually moving to motor angle {desiredMotorAngle}")
    for interAngle in intermediateAngles:
        exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)

# def gradual_motion_until_angle(motorID, currentMotorAngle, desiredWristAngle):
#     exo.enable_motor(motor_id=motorID)
#     step_angle = 0.5
#     print(f"gradually moving to wrist angle {desiredWristAngle}")
#     time.sleep(1.5)

#     moving = True
#     while moving:
#         newAngle = currentMotorAngle + step_angle
#         exo.set_motor_angle(motor_id=motorID, angle=newAngle)
#         time.sleep(0.01)
#         currentWristAngle = exo.get_imu_heading()
#         print(currentWristAngle)
#         if abs(currentWristAngle - desiredWristAngle) <= 1:         # consider proportional motor implementation
#             moving = False
#         else:
#             currentMotorAngle = currentMotorAngle + step_angle

def gradual_flexion_until_angle(motorID, desiredWristAngle):
    exo.enable_motor(motor_id=motorID)
    step_angle = 0.5
    print(f"gradually moving to wrist angle {desiredWristAngle}")
    currentMotorAngle = exo.get_motor_angle(motor_id = motorID)
    time.sleep(1.5)

    moving = True
    while moving:
        newAngle = currentMotorAngle + step_angle
        exo.set_motor_angle(motor_id=motorID, angle=newAngle)
        time.sleep(0.01)
        currentWristAngle = exo.get_imu_heading()
        print(currentWristAngle)
        if (abs(currentWristAngle - desiredWristAngle) <= 1.0):         # consider proportional motor implementation
            moving = False
        else:
            currentMotorAngle = currentMotorAngle + step_angle

def gradual_extension_until_angle(motorID, desiredWristAngle):
    exo.enable_motor(motor_id=motorID)
    step_angle = 0.5
    print(f"gradually moving to wrist angle {desiredWristAngle}")
    currentMotorAngle = exo.get_motor_angle(motor_id = motorID)
    time.sleep(1.5)

    moving = True
    while moving:
        newAngle = currentMotorAngle - step_angle
        exo.set_motor_angle(motor_id=motorID, angle=newAngle)
        time.sleep(0.01)
        currentWristAngle = exo.get_imu_heading()
        print(currentWristAngle)
        if abs(currentWristAngle - desiredWristAngle) <= 1.0:         # consider proportional motor implementation
            moving = False
        else:
            currentMotorAngle = currentMotorAngle - step_angle

# def gradual_motion_to_wrist_angle(motorID, target_wrist_angle):
#     """
#     Moves the motor gradually until IMU yaw reaches target_wrist_angle.
#     """
#     step_motor_angle = 0.5  # degrees per step (motor space)
#     max_attempts = 500
#     exo.enable_motor(motor_id=motorID)
    
#     attempts = 0
#     while attempts < max_attempts:
#         current_wrist_angle = exo.get_imu_heading()
#         print(f"Current wrist angle: {current_wrist_angle:.2f}, Target: {target_wrist_angle:.2f}")
#         if abs(current_wrist_angle - target_wrist_angle) < 0.5:  # close enough
#             break
#         # Increment motor to keep moving toward target
#         current_motor_angle = exo.get_motor_angle(motor_id=motorID)
#         direction = np.sign(target_wrist_angle - current_wrist_angle)
#         next_motor_angle = current_motor_angle + direction * step_motor_angle
#         exo.set_motor_angle(motor_id=motorID, angle=next_motor_angle)
#         time.sleep(0.01)
#         attempts += 1

    


############### Manually Control Assistance: Thread Definition ############### TODO: confirm whether leveraging motor angles is sufficient. Can't think of why we'd need the wrist angles too, since the whole output is related to perceived state and not exact wrist angle. Only exception is if there's enough allowance in design such that same motor angle can give different wrist angles 

def control_structure():


    '''Calibration Stage'''
    exo.reboot_motor(motor_id=motorID)

    # for i in range(100):
    #     rpy = exo.get_imu_heading()
    #     # print(rpy)
    #     time.sleep(0.01)

    input("Place wrist in a relaxed position. Press Enter to record resting wrist angle.")
    # imu_update_enabled.clear()
    # time.sleep(0.1)
    zero_motor_angle = exo.get_motor_angle(motor_id=motorID)
    print(f"rest motor angle: {zero_motor_angle}")
    # imu_update_enabled.set()
    zero_wrist_angle = exo.get_imu_heading()
    print(f"zeros wrist angle: {zero_wrist_angle}")

    input("Move wrist to max flexion. Press Enter to record.")
    max_flexion_wrist_angle = exo.get_imu_heading()
    print(f"Max flexion wrist angle: {max_flexion_wrist_angle:.2f}")
    max_motor_angle = exo.get_motor_angle(motor_id=motorID)
    print(f"max motor angle: {max_motor_angle}")


    if max_flexion_wrist_angle > zero_wrist_angle:
        print("crossed 360 threshold")
        memory_range = (360-max_flexion_wrist_angle) + zero_wrist_angle
    else:
        print("did not cross 360 threshold")
        memory_range = abs(max_flexion_wrist_angle - zero_wrist_angle)
    memory_angleA_difference = (memory_range * 0.31)
    memory_angleB_difference = (memory_range * 0.62)
    memory_angleC_difference = (memory_range * 0.89)
    mem_angle_diffs = [memory_angleA_difference, memory_angleB_difference, memory_angleC_difference]

    fake_memory_multipliers = [0.14, 0.09, 0.46, 0.42, 0.7]
    fake_memory_angle_differences = [(memory_range * m) for m in fake_memory_multipliers]

    print(f"Memory Angle Differences: A={memory_angleA_difference:.2f}, B={memory_angleB_difference:.2f}, C={memory_angleC_difference:.2f}")



    '''Encoding of Memory Angles'''
    wait = input(f"Prepare for memory angle encoding ({encoding_round_count} repetitions). Start in neutral wrist poisition, and hit Enter when ready") 

    for i in range(encoding_round_count):
        for j, mem_ang_diff in enumerate(mem_angle_diffs):
            

            print(f"Angle {j}")
            exo.enable_motor(motor_id=motorID)
            startWristAngle = exo.get_imu_heading()
            targetAngle = startWristAngle - mem_ang_diff
            if targetAngle < 0:
                print("adjusted target angle for 360 cross")
                targetAngle = targetAngle + 360
            print(f"target angle {j}:", targetAngle)
            gradual_flexion_until_angle(motorID, targetAngle)
            time.sleep(2)
            gradual_extension_until_angle(motorID, startWristAngle)
            time.sleep(1)
            


            currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
            if currentMotorAngle > zero_motor_angle:
                gradual_motion(motorID, currentMotorAngle, zero_motor_angle)
        

        # print("Angle A")
        # exo.enable_motor(motor_id=motorID)
        # targetAngle = get_latest_imu_angle()["yaw"] + memory_angleA_difference
        # print("target angle A:", targetAngle)
        # gradual_motion_until_angle(motorID=motorID, currentMotorAngle=zero_motor_angle, desiredWristAngle=targetAngle)
        # time.sleep(2)
        # imu_update_enabled.clear()
        # time.sleep(0.2)
        # currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        # imu_update_enabled.set()
        # gradual_motion(motorID=motorID, currentMotorAngle=currentMotorAngle, desiredMotorAngle=zero_motor_angle)
        # time.sleep(1)

        # print("Angle B")
        # exo.enable_motor(motor_id=motorID)
        # targetAngle = get_latest_imu_angle()["yaw"] + memory_angleB_difference
        # print("target angle B:", targetAngle)
        # gradual_motion_until_angle(motorID=motorID, currentMotorAngle=zero_motor_angle, desiredWristAngle=targetAngle)
        # time.sleep(2)
        # imu_update_enabled.clear()
        # time.sleep(0.2)
        # currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        # imu_update_enabled.set()
        # gradual_motion(motorID=motorID, currentMotorAngle=currentMotorAngle, desiredMotorAngle=zero_motor_angle)
        # time.sleep(1)

        # print("Angle C")
        # exo.enable_motor(motor_id=motorID)
        # targetAngle = get_latest_imu_angle()["yaw"] + memory_angleC_difference
        # print("target angle C:", targetAngle)
        # gradual_motion_until_angle(motorID=motorID, currentMotorAngle=zero_motor_angle, desiredWristAngle=targetAngle)
        # time.sleep(2)
        # imu_update_enabled.clear()
        # time.sleep(0.2)
        # currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        # imu_update_enabled.set()
        # gradual_motion(motorID=motorID, currentMotorAngle=currentMotorAngle, desiredMotorAngle=zero_motor_angle)
        # time.sleep(1)



    '''Control for Passive Intervention'''
    # exo.disable_motor(motor_id=motorID)
    wait = input(f"Prepare for test angle trials ({trial_count} repetitions). Place arm around original neutral position. Press any key to begin trials.")
    
    reps_per_angle = trial_count / 6
    
    trial_sequence = ['A'] * int(reps_per_angle) + ['B'] * int(reps_per_angle) + ['C'] * int(reps_per_angle) + ['D'] * (int(reps_per_angle) * 3)
    random.shuffle(trial_sequence)

    hits_and_misses = []
    answers = {"A": 0, "B": 0, "C": 0, "D": 0}

    for i, trial in enumerate(trial_sequence):

        # if trial in ['A', 'B', 'C']:
        #     target_angle = {"A": memory_angleA, "B": memory_angleB, "C": memory_angleC}[trial]
        # else:
        #     target_angle = random.choice(fake_memory_angles)

        # exo.enable_motor(motor_id=motorID)
        # gradual_motion(motorID, zero_wrist_angle, memory_angleA)
        # time.sleep(2)
        # gradual_motion(motorID, memory_angleA, zero_wrist_angle)
        # time.sleep(1)


        if trial == 'A':
            print("Angle A")
            memory_angle_difference = memory_angleA_difference     
        elif trial == 'B':
            print("Angle B")
            memory_angle_difference = memory_angleB_difference
        elif trial == 'C':
            print("Angle C")
            memory_angle_difference = memory_angleC_difference
        elif trial == 'D':
            print("Non-memory angle")
            memory_angle_difference = random.choice(fake_memory_angle_differences)


# print(f"Angle {j}")
# exo.enable_motor(motor_id=motorID)
# startWristAngle = exo.get_imu_heading()
# targetAngle = startWristAngle - mem_ang_diff
# if targetAngle < 0:
#     print("adjusted target angle for 360 cross")
#     targetAngle = targetAngle + 360
# print(f"target angle {j}:", targetAngle)
# gradual_flexion_until_angle(motorID, targetAngle)
# time.sleep(2)
# gradual_extension_until_angle(motorID, startWristAngle)
# time.sleep(1)



# currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
# if currentMotorAngle > zero_motor_angle:
#     gradual_motion(motorID, currentMotorAngle, zero_motor_angle)

        exo.enable_motor(motor_id=motorID)
        startAngle = exo.get_imu_heading()
        targetAngle = startAngle - memory_angle_difference
        if targetAngle < 0:
            print("adjusted target angle for 360 cross")
            targetAngle = targetAngle + 360
        print(f"target angle {trial}:", targetAngle)

        gradual_flexion_until_angle(motorID, targetAngle)
        time.sleep(2)
        # currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        gradual_extension_until_angle(motorID, startAngle)
        time.sleep(1)

        currentMotorAngle = exo.get_motor_angle(motor_id=motorID)
        if currentMotorAngle > zero_motor_angle:
            gradual_motion(motorID, currentMotorAngle, zero_motor_angle)

        angleGuess = input(f"Was the angle (#{i}) in the list? If so, which one was it? Enter 'a' for angleA, 'b' for angleB, 'c' for angleC, or 'd' for a fake memory angle.").upper()
        if angleGuess=='Z':
            print("early out")
            break


        if angleGuess == trial:
            hits_and_misses.append((trial, angleGuess, 1))
        else:
            hits_and_misses.append((trial, angleGuess, 0))

        if angleGuess in ['A', 'B', 'C', 'D']:
            answers[angleGuess] += 1

        

    with open(f"stmGuesses_{filename_timestamp}.txt", 'w') as guessFile:
        guessFile.write(" ".join(map(str, hits_and_misses)))
        guessFile.write("\n Answer Dictionary")
        guessFile.write(str(answers))


    stop_event.set()



############### Main Function: Glorified Thread Manager ###############

if __name__ == "__main__":

    filename_timestamp = int(time.time())

    trigger_timestamps = []

    stop_event = threading.Event()

    # t1 = threading.Thread(target=record_data)
    # t_imu = threading.Thread(target=imu_manager)
    t_control = threading.Thread(target=control_structure)

    

    # t_imu.start()
    t_control.start()

    # t_imu.join()
    t_control.join()

    print("End of the line")



 