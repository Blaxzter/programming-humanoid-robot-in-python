import matplotlib.pyplot as plt
import numpy as np

time = []
act_y = {'LWristYaw': [], 'RWristYaw': [], 'HeadYaw': [], 'RHipPitch': [], 'RElbowYaw': [], 'RHand': [], 'RShoulderPitch': [], 'LShoulderRoll': [], 'LHand': [], 'LKneePitch': [], 'RAnkleRoll': [], 'LShoulderPitch': [], 'LHipPitch': [], 'LElbowYaw': [], 'LAnklePitch': [], 'HeadPitch': [], 'LElbowRoll': [], 'RShoulderRoll': [], 'LAnkleRoll': [], 'LHipYawPitch': [], 'RAnklePitch': [], 'LHipRoll': [], 'RHipRoll': [], 'RElbowRoll': [], 'RKneePitch': []}
key_y = {'LWristYaw': [], 'RWristYaw': [], 'HeadYaw': [], 'RHipPitch': [], 'RElbowYaw': [], 'RHand': [], 'RShoulderPitch': [], 'LShoulderRoll': [], 'LHand': [], 'LKneePitch': [], 'RAnkleRoll': [], 'LShoulderPitch': [], 'LHipPitch': [], 'LElbowYaw': [], 'LAnklePitch': [], 'HeadPitch': [], 'LElbowRoll': [], 'RShoulderRoll': [], 'LAnkleRoll': [], 'LHipYawPitch': [], 'RAnklePitch': [], 'LHipRoll': [], 'RHipRoll': [], 'RElbowRoll': [], 'RKneePitch': []}

time.insert(0, 0)

adder = 0
with open("Actual.txt", "r") as act_file:
    for line in act_file:
        parts = line.rsplit(' ')
        parts[-1] = parts[-1][0:-1]
        # print len(time), '<', int(parts[0])
        if len(time) < int(parts[0]):
            if time[-1] > float(parts[2]) + adder:
                adder = time[-1]
            # print time[-1], "<", int(parts[0])
            time.insert(int(parts[0]), float(parts[2]) + adder)
        act_y[parts[1]].insert(int(parts[0]), float(parts[3]))

with open("KeyFrame.txt", "r") as act_file:
    for line in act_file:
        parts = line.rsplit(' ')
        parts[-1] = parts[-1][0:-1]
        key_y[parts[1]].insert(int(parts[0]), float(parts[3]))

for i in range(len(act_y.keys())):
    plt.figure(i+1)
    plt.title(str(i) + " " + act_y.keys()[i])

    plt.plot(time[0:len(act_y[act_y.keys()[i]])], act_y[act_y.keys()[i]][0:len(time)], color="blue", linestyle="-")
    plt.plot(time[0:len(key_y[key_y.keys()[i]])], key_y[key_y.keys()[i]][0:len(time)], color="green", linestyle="-")
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()

plt.show()
