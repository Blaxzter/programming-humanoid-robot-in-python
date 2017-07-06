'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, sin, cos
import numpy as np

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain

        # We dont need no hands we are strong independet robots who no need no hands and wrists
        # i can cut my onions with my feet anyways
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'LAnkleRoll']
                       }

        # Joint length from http://doc.aldebaran.com/2-1/family/robots/links_robot.html
        # In x y z form
        self.jointLengths = {'HeadYaw': (0, 0, 126.5), 'HeadPitch': (0, 0, 0),  # Head Chain
                             # Left arm Chain
                             'LShoulderPitch': (0, 98, 100), 'LShoulderRoll': (0, 0, 0), 'LElbowYaw': (105, 15, 0),
                             'LElbowRoll': (0, 0, 0), 'LWristYaw': (55.95, 0, 0),
                             # Right arm Chain, the x is inverted due to the picture on the page mentioned above
                             'RShoulderPitch': (0, -98, 100), 'RShoulderRoll': (0, 0, 0), 'RElbowYaw': (105, -15, 0),
                             'RElbowRoll': (0, 0, 0), 'RWristYaw': (55.95, 0, 0),
                             # Left leg Chain
                             'LHipYawPitch': (0, 50, -85), 'LHipRoll': (0, 0, 0), 'LHipPitch': (0, 0, 0),
                             'LKneePitch': (0, 0, -100), 'LAnklePitch': (0, 0, -102.9), 'LAnkleRoll': (0, 0, 0),
                             # Right leg Chain
                             'RHipYawPitch': (0, -50, -85), 'RHipRoll': (0, 0, 0), 'RHipPitch': (0, 0, 0),
                             'RKneePitch': (0, 0, -100), 'RAnklePitch': (0, 0, -102.9), 'RAnkleRoll': (0, 0, 0)
                             }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        T = identity(4)

        s = sin(joint_angle)
        c = cos(joint_angle)

        # Like it's stated in B-Human coding guide here we apply the 45 degree rotation of the LHipYawPitch and RHipYawPitch
        if 'LHipYawPitch' == joint_name or 'RHipYawPitch' == joint_name:
            if joint_name[0] == 'L':
                T = np.dot(T, np.array([[1, 0, 0, 0], [0, cos(np.pi/4), -sin(np.pi/4), 0], [0, sin(np.pi/4), cos(np.pi/4), 0], [0, 0, 0, 1]]))
            if joint_name[0] == 'R':
                T = np.dot(T, np.array([[1, 0, 0, 0], [0, cos(-np.pi/4), -sin(-np.pi/4), 0], [0, sin(-np.pi/4), cos(-np.pi/4), 0], [0, 0, 0, 1]]))

        # Differ between the joint angles for Roll, Pitch and Yaw movement
        if 'Roll' in joint_name:
            T = np.dot(T, np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]))
        if 'Pitch' in joint_name:
            T = np.dot(T, np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]))
        if 'Yaw' in joint_name:
            T = np.dot(T, np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))

        # I decided to put the x y z kords into the last row like it's done in the robot_arm_2d notebook
        # even so its different shown on page 36 of the third lecture material
        if joint_name in self.jointLengths.keys():
            for i in range(3): T[i, -1] = self.jointLengths[joint_name][i]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        T = identity(4)
        for joint in joints.keys():
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            T = np.dot(T, Tl)
            self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
