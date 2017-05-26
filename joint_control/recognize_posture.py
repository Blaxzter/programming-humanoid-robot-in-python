'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
from sklearn import svm, metrics
import pickle, numpy as np

ROBOT_POSE_CLF = 'robot_pose.pkl'
features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
classes = ['Sit', 'Left', 'Knee', 'Frog', 'Right', 'Crouch', 'StandInit', 'Stand', 'Back', 'Belly', 'HeadBack']

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        clf2 = pickle.load(open(ROBOT_POSE_CLF))
        n = len(features) + 2
        detect_data = np.ndarray((n, 1))
        for i in enumerate(features):
            detect_data[i[0]] = perception.joint[i[1]]

        detect_data[-2] = perception.imu[0]
        detect_data[-1] = perception.imu[1]
        posture = classes[int(clf2.predict(detect_data.T)[0])]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
