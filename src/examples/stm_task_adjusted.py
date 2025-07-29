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

stream_name = 'SAGA'     #TODO: vary stream name so that we don't pick up the wrong device       # Search for active stream names using names_of_active_lsl_streams.py and connect to NML wifi

encoding_round_count = 3     # 3 exposures per memory angle  (chiyohara 2020)
trial_count = 60      # Number of trials (MUST BE A MULTIPLE OF 6) 

###############  Data Recording ############### 

recorded_data = []
recorded_timestamps = []


###############  Connect to LSL Stream ############### # won't need the emg data during the stm task

# print("Resolving LSL stream...")
# #streams = resolve_stream('name', stream_name)
# streams = pylsl.resolve_streams()

# inlet = None
# for i, stream in enumerate(streams):
#     if stream.name() == stream_name:
#         print(f"Found stream {i+1}: {stream.name()} (Type: {stream.type()})")
#         inlet = StreamInlet(stream, max_buflen=60)
#         break

# info = inlet.info()
# n_channels = info.channel_count()

# print(f"Connected to stream: {stream_name}")
# print(f"Detected {n_channels} channels.")

# # Get channel labels
# channel_labels = []
# try:
#     chns = info.desc().child("channels").child("channel")
#     for _ in range(n_channels):
#         label = chns.child_value("label")
#         channel_labels.append(label if label else f"ch{_}")
#         chns = chns.next_sibling()
# except Exception:
#     print("!!!!!!! No channel names found. Using default labels.")
#     channel_labels = [f"ch{i}" for i in range(n_channels)]
 
# print("Channel labels:", channel_labels)


############### Backup for commands not in helper class ###############

# def send_command(exo, msg):
#     """Send a command to the exoskeleton."""
#     if exo:
#         try:
#             if not msg.endswith('\n'):
#                 msg = msg + '\n'
#             exo.write(msg.encode('utf-8'))
#             print(f"Command sent: {msg.strip()}")
#             time.sleep(0.01)
#         except Exception as e:
#             print(f"Error sending command: {e}")
#     else:
#         print("Exoskeleton not connected, command not sent.")



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

# def record_data(): 
#     filename = f"lsl_record_{filename_timestamp}.csv"
#     with open(filename, mode = 'w', newline='') as file:
#         writer = csv.writer(file)
#         print("Recording streamed data")

#         while not stop_event.is_set():
#             sample, timestamp = inlet.pull_sample()
#             writer.writerow([timestamp] + sample)
#         print("\n Recording stopped. Data saved!")


############### Gradual Motion Function ############### 

def gradual_motion(motorID, currentAngle, desiredAngle):
    exo.enable_motor(motor_id=motorID)
    intermediateAngles = np.linspace(currentAngle, desiredAngle, 100)
    print("gradually moving")
    for interAngle in intermediateAngles:
        exo.set_motor_angle(motor_id=motorID, angle=interAngle)
        time.sleep(0.01)
    


############### Manually Control Assistance: Thread Definition ############### TODO: confirm whether leveraging motor angles is sufficient. Can't think of why we'd need the wrist angles too, since the whole output is related to perceived state and not exact wrist angle. Only exception is if there's enough allowance in design such that same motor angle can give different wrist angles 

def control_structure():


    '''Calibration Stage'''
    exo.reboot_motor(motor_id=motorID)

    wait = input("Place wrist in a relaxed positioin with the forearm supported, such that the wrist is naturally flexed by gravity. Press any key to record the motor angle.")
    rest_angle = exo.get_motor_angle(motor_id=motorID)

    wait = input("Move patients wrist to be as extended as possible, whether the limit is physiological or mechanical. This will define the bounds of the memory angles.\n Hit any key to record the motor angle.")
    max_extension_angle = exo.get_motor_angle(motor_id=motorID)

    memory_range = abs(max_extension_angle) + abs(rest_angle)
    memory_angleA = rest_angle + (memory_range * 0.31)
    memory_angleB = rest_angle + (memory_range * 0.62)
    memory_angleC = rest_angle + (memory_range * 0.89)

    fake_memory_angle_multipliers = [0.14, 0.09, 0.46, 0.42, 0.7]
    fake_memory_angle_multipliers = np.array(fake_memory_angle_multipliers)
    fake_memory_angles = rest_angle + (memory_range * fake_memory_angle_multipliers)

    print(f"rest: {rest_angle}, memoryA: {memory_angleA}, memoryB: {memory_angleB}, memoryC: {memory_angleC}")


    '''Encoding of Memory Angles'''
    wait = input(f"Prepare for memory angle encoding ({encoding_round_count} repetitions). Start in neutral wrist poisition, and hit Enter when ready") 

    for i in range(encoding_round_count):
        print("Angle A")
        exo.enable_motor(motor_id=motorID)
        gradual_motion(motorID, rest_angle, memory_angleA)
        time.sleep(2)
        gradual_motion(motorID, memory_angleA, rest_angle)
        time.sleep(1)

        print("Angle B")
        exo.enable_motor(motor_id=motorID)
        gradual_motion(motorID, rest_angle, memory_angleB)
        time.sleep(2)
        gradual_motion(motorID, memory_angleB, rest_angle)
        time.sleep(1)

        print("Angle C")
        exo.enable_motor(motor_id=motorID)
        gradual_motion(motorID, rest_angle, memory_angleC)
        time.sleep(2)
        gradual_motion(motorID, memory_angleC, rest_angle)
        time.sleep(1)



    '''Control for Passive Intervention'''
    wait = input(f"Prepare for test angle trials ({trial_count} repetitions). Place arm around original neutral position. Press any key to begin trials.")
    
    reps_per_angle = trial_count / 6
    
    trial_sequence = ['A'] * int(reps_per_angle) + ['B'] * int(reps_per_angle) + ['C'] * int(reps_per_angle) + ['D'] * (int(reps_per_angle) * 3)
    random.shuffle(trial_sequence)

    hits_and_misses = []

    answers = {
            "A" : 0,
            "B" : 0,
            "C" : 0,
            "D" : 0
        }

    for trial in trial_sequence:
        if trial == 'A':
            print("Angle A")
            exo.enable_motor(motor_id=motorID)
            gradual_motion(motorID, rest_angle, memory_angleA)
            time.sleep(2)
            gradual_motion(motorID, memory_angleA, rest_angle)
            time.sleep(1)
        elif trial == 'B':
            print("Angle B")
            exo.enable_motor(motor_id=motorID)
            gradual_motion(motorID, rest_angle, memory_angleB)
            time.sleep(2)
            gradual_motion(motorID, memory_angleB, rest_angle)
            time.sleep(1)
        elif trial == 'C':
            print("Angle C")
            exo.enable_motor(motor_id=motorID)
            gradual_motion(motorID, rest_angle, memory_angleC)
            time.sleep(2)
            gradual_motion(motorID, memory_angleC, rest_angle)
            time.sleep(1)
        elif trial == 'D':
            print("Non-memory angle")
            fake_angle = fake_memory_angles[random.randint(0, (len(fake_memory_angles)-1))]
            exo.enable_motor(motor_id=motorID)
            gradual_motion(motorID, rest_angle, fake_angle)
            time.sleep(2)
            gradual_motion(motorID, fake_angle, rest_angle)
            time.sleep(1)
        else:
            print("if you're reading this something is very wrong... no idea how we would get here")

        angleGuess = input("Was the angle in the list? If so, which one was it? Enter 'a' for angleA, 'b' for angleB, 'c' for angleC, or 'd' for a fake memory angle.")
        angleGuess = angleGuess.upper()


        if angleGuess=='Z':
            print("early out")
            break


        if angleGuess == trial:
            hits_and_misses.append((trial, 1))
        else:
            hits_and_misses.append((trial, 0))

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

    # t1 = threading.Thread(target=record_data)
    t2 = threading.Thread(target=control_structure)

    stop_event = threading.Event()

    # t1.start()
    t2.start()

    # t1.join()
    t2.join()

    print("End of the line")



