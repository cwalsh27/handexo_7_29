import random

trial_count = 60


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
       
    elif trial == 'B':
        print("Angle B")
       
    elif trial == 'C':
        print("Angle C")
       
    elif trial == 'D':
        print("Non-memory angle")
       


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

    

with open(f"stmGuesses_0.txt", 'w') as guessFile:
    guessFile.write(" ".join(map(str, hits_and_misses)))
    guessFile.write("\n Answer Dictionary")
    guessFile.write(str(answers))


