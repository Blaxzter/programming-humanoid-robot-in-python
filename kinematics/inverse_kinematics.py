'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from math import atan2
from collections import OrderedDict
from numpy import matrix
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        print "START"

        # Using the easyer numerical inverse kinematics
        joint_angles = np.asarray([0.0] * len(self.chains[effector_name]))
        lambda_ = 0.001


        current_effected_joints = OrderedDict()
        for name in self.chains[effector_name]:
            current_effected_joints[name] = self.perception.joint[name]

        target_effector = self.chains[effector_name][-1]
        target = self.from_trans(transform)

        while True:
            self.forward_kinematics(current_effected_joints)  # calc current forward kinematics from current joint

            T = [0] * len(self.chains[effector_name])
            for i, name in enumerate(self.chains[effector_name]):
                T[i] = self.transforms[name]  # get the solution

            Te = np.array([self.from_trans(T[-1])])
            e = target - Te
            T = np.array([self.from_trans(i) for i in T[0:len(self.chains[effector_name])]])
            J = Te - T
            J = J.T
            J[-1, :] = 1  # angular velocity
            JJT = np.dot(J, J.T)
            d_theta = lambda_ * np.dot(np.dot(J.T, np.linalg.pinv(JJT)), e.T)

            for i, name in enumerate(self.chains[effector_name]):
                current_effected_joints[name] += np.asarray(d_theta.T)[0][i]

            print np.linalg.norm(d_theta), current_effected_joints, e
            if np.linalg.norm(d_theta) < 1e-3:
                print '\n ---------------------------- break ----------------------------- \n'
                break

        return current_effected_joints

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # Solve the inverese kinematics and get the angles

        joint_angles = self.inverse_kinematics(effector_name, transform)

        # put the returned angles into the keyframe
        names = self.chains[effector_name]
        times = [[0, 5]] * len(names)
        keys = []
        for i, name in enumerate(names):
            print name, "\t", joint_angles[name]
            keys.insert(i, [[self.perception.joint[name], [3, 0, 0]], [joint_angles[name], [3, 0, 0]]])


        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

    def from_trans(self, T):
        # return x,y,z
        x, y, z = T[0, -1], T[1, -1], T[2, -1]

        theta_x, theta_y, theta_z = 0, 0, 0

        if T[0, 0] == 1:
            theta_x = atan2(T[2, 1], T[1, 1])
        elif T[1, 1] == 1:
            theta_y = atan2(T[0, 2], T[0, 0])
        elif T[2, 2] == 1:
            theta_z = atan2(T[1, 0], T[0, 0])

        return np.array([x, y, z, theta_x, theta_y, theta_z])


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[0, -1] = 0
    T[1, -1] = -93
    T[2, -1] = -228
    agent.set_transforms('LLeg', T)
    agent.run()
