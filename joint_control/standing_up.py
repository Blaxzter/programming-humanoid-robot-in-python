'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from __future__ import print_function
from recognize_posture import PostureRecognitionAgent
from keyframes import *

startTime = 0

class StandingUpAgent(PostureRecognitionAgent):

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        global startTime
        if startTime == 0:
            startTime = self.perception.time

        print(self.perception.time - startTime, ": ", end="")
        posture = self.posture
        print(posture, end="")

        self.keyframes = ([], [], [])
        if self.perception.time - startTime > 2.6:
            print(" GO ", end="")
            if posture == 'Belly':
                self.keyframes = leftBellyToStand()

            if posture == 'Back':
                self.keyframes = rightBackToStand()
        else:
            print(" Wait ", end="")

        """
        start_keyframes = self.createPostureKeyframes(self.keyframes, self.perception)
        self.start_posture_Splines = self.cubic_spline_interpolation(start_keyframes)
        self.at_startPosture = 0
        
        # Not the right place to bring the robi to the starting position
        if self.at_startPosture == 0:
            used_taget_splines = self.start_posture_Splines
            if self.current_time >= 2:
                self.at_startPosture = 1
                self.startTime = perception.time """

class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds


    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
