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


from pid import PIDAgent
from keyframes import wipe_forehead, hello
import numpy as np

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.saved_targed_splines = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.startTime = 0
        self.saved_keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # Check for new keyframes to interpolate (only interpolate once every keyframe)
        if self.saved_keyframes != keyframes:
            # Save the keyframe and move start time
            self.saved_keyframes = keyframes
            self.startTime = perception.time

            # interpolate for every joint
            jointCount = 0
            for joint in keyframes[0]:
                # get the times or the x values
                times = keyframes[1][jointCount]
                length = len(times) - 1

                # get the y values out of the keyframes for the spine interpolation
                y = np.zeros([length + 1])
                for i in range(length + 1):
                    y[i] = keyframes[2][jointCount][i][0]

                # create the matrix for solving purpose
                x_mat = np.zeros([4 * length, 4 * length])
                y_mat = np.zeros([4 * length, 1])

                # fill the created matrix with information about the keyframes
                for val in range(length-1):
                    a = 4 * val
                    line = a + 1
                    x_mat[line, a:a + 4] = np.array([1, times[val], times[val] ** 2, times[val] ** 3])
                    x_mat[line + 1, a:a + 4] = np.array([1, times[val + 1], times[val + 1] ** 2, times[val + 1] ** 3])
                    x_mat[line + 2, a:a + 8] = np.array([0, 1, 2 * times[val + 1], 3 * times[val + 1] ** 2, 0, -1, -2 * times[val + 1], -3 * times[val + 1] ** 2])
                    x_mat[line + 3, a:a + 8] = np.array([0, 0, 2, 6 * times[val + 1], 0, 0, -2, -6 * times[val + 1]])

                    y_mat[line:line + 4, 0] = np.array([y[val], y[val + 1], 0, 0])

                # fill the corner cases where f'(0) = f'(n-1) = 0
                x_mat[0, 0:4] = np.array([0, 0, 2, 6 * times[0]])
                x_mat[4 * length - 3, 4 * length - 4:4 * length] = np.array([1, times[length - 1], times[length - 1] ** 2, times[length - 1] ** 3])
                x_mat[4 * length - 2, 4 * length - 4:4 * length] = np.array([1, times[length], times[length] ** 2, times[length] ** 3])
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
                    splines.append([times[i], times[i+1], poly]) # save the polynom with the times of the polynom

                # Save the polynoms of the joint
                self.saved_targed_splines[jointCount] = [joint, splines]
                jointCount += 1

        # get the right return angle
        currentTime = perception.time - self.startTime
        for v in self.saved_targed_splines:
            sens_value = 0

            if currentTime > v[1][-1][1]:
                sens_value = v[1][-1][2](v[1][-1][1])
            else:
                for time1, time2, pol in v[1]:
                    if time2 < currentTime > time1:
                        sens_value = pol(currentTime)

            target_joints[v[0]] = sens_value

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
