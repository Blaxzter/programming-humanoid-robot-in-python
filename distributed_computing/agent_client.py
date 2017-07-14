'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib
import threading
import sys
import os
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from keyframes import hello

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def keyFrame_thread(self, keyframes):
        self.proxy.execute_keyframes(keyframes)

    def set_transform_thread(self, effector_name, transform):
        self.proxy.set_transform(effector_name, transform)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        thread = threading.Thread(target=self.keyFrame_thread, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        thread = threading.Thread(target=self.set_transform_thread, args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.MyProxy = xmlrpclib.ServerProxy("http://localhost:9000")
        self.post = PostHandler(self.MyProxy)

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.MyProxy.get_angle(joint_name)
        self.post = PostHandler(self)

    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.MyProxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # That sadly cant work because the recognize posture agent isn't part of the inheritance list
        return self.MyProxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.post.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.MyProxy.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.post.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    print agent.get_transform("LLeg")

