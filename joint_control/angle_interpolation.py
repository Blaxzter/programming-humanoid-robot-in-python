'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from __future__ import print_function
import numpy as np
from pid import PIDAgent
from keyframes import wipe_forehead, hello
import matplotlib.pyplot as plt
import copy


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)

        self.write_file = False
        self.plot = False

        self.keyframes = ([], [], [])
        self.interpolated = 0
        self.startTime = -1
        self.firstTime = -1
        self.endTime = -1
        self.at_startPosture = 1
        self.start_posture_Splines = ([], [], [])
        self.saved_target_splines = {}
        self.interpolated_keyframes = ([], [], [])
        self.current_time = 0
        self.counter = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #if len(target_joints) != 0:
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        if keyframes == ([], [], []) and self.endTime < self.current_time:
            print("Idle Mode ", end="")
            # target_joints = perception.joint
        else:
            if self.interpolated == 0 or (keyframes != self.interpolated_keyframes and keyframes != ([], [], [])):
                self.startTime = perception.time
                target_joints = perception.joint
                self.interpolated_keyframes = keyframes
                # self.saved_target_splines = self.cubic_spline_interpolation(copy.deepcopy(keyframes))
                self.saved_target_splines = self.hermite_interpolation(copy.deepcopy(keyframes))
                self.interpolated = 1
                self.endTime = self.get_latest_endTime()

                if self.plot:
                    self.show_plot()

            self.current_time = perception.time - self.startTime
            # print("Current Time:", self.current_time)

            for joint_name, spline_list in self.saved_target_splines.iteritems():
                # get the right joint angle
                for time1, time2, spline in spline_list:
                    if self.current_time >= time1 and self.current_time < time2:
                        if self.write_file:
                            with open("KeyFrame.txt", "a") as save_file:
                                save = str(self.counter) + " " + str(joint_name) + ' ' + str(self.current_time) + ' ' + str(spline(self.current_time)) + '\n'
                                save_file.write(save)
                        target_joints[joint_name] = spline(self.current_time)
                        break

                if self.write_file:
                    with open("Actual.txt", "a") as save_file:
                        if joint_name in perception.joint.keys():
                            save = str(self.counter) + " " + str(joint_name) + ' ' + str(self.current_time) + ' ' + str(
                                perception.joint[joint_name]) + '\n'
                        else:
                            save = str(self.counter) + " " + str(joint_name) + ' ' + str(self.current_time) + ' ' + str(
                                0) + '\n'
                        save_file.write(save)

            self.counter += 1

            # After the last keyframe (time) he should be standing (no need for interpolation)
            if self.current_time > self.endTime:
                self.interpolated = 0
                self.startTime = -1
                self.firstTime = -1
                self.endTime = -1
                self.keyframes = ([], [], [])

        print(target_joints)
        if 'LHipYawPitch' in target_joints.keys():
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']

        return target_joints

    def show_plot(self):
        x = np.arange(0., self.endTime, 0.01)
        x_1 = {}
        y_0 = {}
        y_1 = {}
        for joint_name in self.keyframes[0]:
            if joint_name not in x_1.keys():
                x_1[joint_name] = []
                y_1[joint_name] = []
            joint_id = self.keyframes[0].index(joint_name)
            for time in self.keyframes[1][joint_id]:
                x_1[joint_name].append(time)
            for angle in self.keyframes[2][joint_id]:
                y_1[joint_name].append(angle[0])

        for i in x:
            for joint_name, spline_list in self.saved_target_splines.iteritems():
                if joint_name not in y_0.keys():
                    y_0[joint_name] = []
                for time1, time2, spline in spline_list:
                    if i < spline_list[0][0] or i >= spline_list[-1][1]:
                        y_0[joint_name].append(y_0[joint_name][-1])
                        break
                    if time2 > i >= time1:
                        y_0[joint_name].append(spline(i))
                        break

        for i in range(len(y_0.keys())):
            plt.figure(i + 1)
            joint_name = y_0.keys()[i]
            plt.title(str(i) + " " + joint_name)
            plt.plot(x, y_0[joint_name], color="blue", linestyle="-")
            plt.plot(x_1[joint_name], y_1[joint_name], color="green", linestyle="-", marker='x')
            # figManager = plt.get_current_fig_manager()
            # figManager.window.showMaximized()

        plt.show()

    def add_posture_to_keyframe(self, keyframes):
        for i in range(len(keyframes[1])):
            keyframes[1][i].insert(0, 0)

        for i, name in enumerate(keyframes[0]):
            if name in self.perception.joint.keys():
                keyframes[2][i].insert(0, [self.perception.joint[name], [], []])
            else:
                keyframes[2][i].insert(0, [0, [], []])

        return keyframes

    def hermite_interpolation(self, keyframes):

        # print("------------------------------------------- Interpolate using Hermite ------------------------------------------- ")

        keyframes = self.add_posture_to_keyframe(keyframes)

        saved_target_splines = {}
        for count, joint_name in enumerate(keyframes[0]):

            # print("---------------------------------", count, joint_name, "---------------------------------")

            x = keyframes[1][count]
            n = len(x)

            y = np.zeros([n])
            for i in range(n):
                y[i] = keyframes[2][count][i][0]

            yp = np.zeros([n])

            for i in range(1, n - 1):
                if (y[i + 1] - y[i]) * (y[i] - y[i - 1]) <= 0:
                    yp[i] = 0
                else:
                    if np.abs((y[i + 1] - y[i]) / (x[i + 1] - x[i])) > 1:
                        yp[i] = 0
                    else:
                        yp[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i])

                    #yp[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1])
                    #v0 = (y[i] - y[i - 1]) / (x[i] - x[i - 1])
                    #v1 = (y[i + 1] - y[i]) / (x[i + 1] - x[i])
                    #yp[i] = (v0 + v1) / 2

            spline = []

            A = np.zeros([4, 4])
            b = np.zeros([4, 1])
            for i in range(n - 1):
                A[0] = np.array([1, x[i], x[i] ** 2, x[i] ** 3])
                A[1] = np.array([1, x[i + 1], x[i + 1] ** 2, x[i + 1] ** 3])
                A[2] = np.array([0, 1, 2 * x[i], 3 * x[i] ** 2])
                A[3] = np.array([0, 1, 2 * x[i + 1], 3 * x[i + 1] ** 2])
                b = np.array([y[i], y[i + 1], yp[i], yp[i + 1]])

                c = np.linalg.solve(A, b).reshape(-1)
                c = c[::-1]
                poly = np.poly1d(c)
                spline.append([x[i], x[i + 1], poly])  # save the polynom with the times of the polynom

            # Save the polynoms of the joint
            name = joint_name
            saved_target_splines[name] = spline

        return saved_target_splines

    def cubic_spline_interpolation(self, keyframes):
        print(" ------------------------- Interpolate ------------------------- ")

        keyframes = self.add_posture_to_keyframe(keyframes)

        saved_target_splines = {}
        # interpolate for every joint
        for joint_name in enumerate(keyframes[0]):
            # get the times of the x values
            times = keyframes[1][joint_name[0]]
            length = len(times) - 1

            # get the y values out of the keyframes for the spine interpolation
            y = np.zeros([length + 1])
            for i in range(length + 1):
                y[i] = keyframes[2][joint_name[0]][i][0]

            # create the matrix for solving purpose
            x_mat = np.zeros([4 * length, 4 * length])
            y_mat = np.zeros([4 * length, 1])

            # fill the created matrix with information about the keyframes
            for val in range(length - 1):
                a = 4 * val
                line = a + 1
                x_mat[line, a:a + 4] = np.array([1, times[val], times[val] ** 2, times[val] ** 3])
                x_mat[line + 1, a:a + 4] = np.array([1, times[val + 1], times[val + 1] ** 2, times[val + 1] ** 3])
                x_mat[line + 2, a:a + 8] = np.array(
                    [0, 1, 2 * times[val + 1], 3 * times[val + 1] ** 2, 0, -1, -2 * times[val + 1],
                     -3 * times[val + 1] ** 2])
                x_mat[line + 3, a:a + 8] = np.array([0, 0, 2, 6 * times[val + 1], 0, 0, -2, -6 * times[val + 1]])

                y_mat[line:line + 4, 0] = np.array([y[val], y[val + 1], 0, 0])

            # fill the corner cases where f'(0) = f'(n-1) = 0
            x_mat[0, 0:4] = np.array([0, 0, 2, 6 * times[0]])
            x_mat[4 * length - 3, 4 * length - 4:4 * length] = np.array(
                [1, times[length - 1], times[length - 1] ** 2, times[length - 1] ** 3])
            x_mat[4 * length - 2, 4 * length - 4:4 * length] = np.array(
                [1, times[length], times[length] ** 2, times[length] ** 3])
            x_mat[4 * length - 1, 4 * length - 4:4 * length] = np.array([0, 0, 2, 6 * times[length]])

            y_mat[(4 * length - 3):(4 * length), 0] = np.array([y[length - 1], y[length], 0])

            # solve the created LGS
            solution = np.linalg.solve(x_mat, y_mat).reshape(-1)

            # get the polynomes out of the solution
            splines = []
            for i in range(length):
                coffs = solution[(4 * i):(4 * i) + 4]
                coffs = coffs[::-1]
                poly = np.poly1d(coffs)
                splines.append([times[i], times[i + 1], poly])  # save the polynom with the times of the polynom

            # Save the polynoms of the joint
            name = joint_name[1]
            saved_target_splines[name] = splines
        return saved_target_splines

    def get_latest_endTime(self):
        latest_end_time = -1
        for joint_name, spline_list in self.saved_target_splines.iteritems():
            if latest_end_time < spline_list[-1][1]:
                latest_end_time = spline_list[-1][1]
        return latest_end_time

    def createPostureKeyframes(self, keyframes, perception):
        duration = 2
        return_keyframe = (keyframes[0], [[0, duration]] * len(keyframes[0]), [])
        for i in range(len(keyframes[0])):
            if keyframes[0][i] in perception.joint.keys():
                return_keyframe[2].insert(i, [[perception.joint[keyframes[0][i]], [3, 0, 0]], keyframes[2][i][0]])
            else:
                return_keyframe[2].insert(i, [[0, [3, 0, 0]], [0, [3, 0, 0]]])
        return return_keyframe


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
