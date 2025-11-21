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
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np
from recognize_posture import PostureRecognitionAgent


def get_initial_transform(chain_name):
    '''Fixed translation from Torso to the first joint of the chain.'''
    # Values derived from http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    TORSO_TRANSFORMS = {
        'Head': (0.0, 0.0, 0.1265),     # Torso -> HeadYaw
        'LArm': (0.0, 0.098, 0.100),    # Torso -> LShoulderPitch
        'RArm': (0.0, -0.098, 0.100),   # Torso -> RShoulderPitch
        'LLeg': (0.0, 0.050, -0.085),   # Torso -> LHipYawPitch
        'RLeg': (0.0, -0.050, -0.085)   # Torso -> RHipYawPitch
    }
    
    if chain_name not in TORSO_TRANSFORMS:
        return identity(4) 
        
    X, Y, Z = TORSO_TRANSFORMS[chain_name]
    T = identity(4)
    # Use individual assignment to avoid ValueError
    T[0, 3] = X
    T[1, 3] = Y
    T[2, 3] = Z
    return T

class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        # ['HeadYaw', 'HeadPitch', 'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
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
        # YOUR CODE HERE

        KINEMATIC_PARAMETERS = {
            # Head
            'HeadYaw':        {'axis': 'Z', 'V': (0.0, 0.0, 0.0)}, 
            'HeadPitch':      {'axis': 'Y', 'V': (0.0, 0.0, 0.0)},
            
            # Left Leg (LHipYawPitch excluded as it is a coupled, complex rotation)
            'LHipRoll':       {'axis': 'X', 'V': (0.0, 0.0, 0.0)},
            'LHipPitch':      {'axis': 'Y', 'V': (0.0, 0.0, -0.100)},      # ThighLength (100mm)
            'LKneePitch':     {'axis': 'Y', 'V': (0.0, 0.0, -0.1029)},     # TibiaLength (102.9mm)
            'LAnklePitch':    {'axis': 'Y', 'V': (0.0, 0.0, 0.0)},
            'LAnkleRoll':     {'axis': 'X', 'V': (0.0, 0.0, -0.04519)},    # FootHeight (45.19mm)

            # Right Leg
            'RHipRoll':       {'axis': 'X', 'V': (0.0, 0.0, 0.0)},
            'RHipPitch':      {'axis': 'Y', 'V': (0.0, 0.0, -0.100)},
            'RKneePitch':     {'axis': 'Y', 'V': (0.0, 0.0, -0.1029)},
            'RAnklePitch':    {'axis': 'Y', 'V': (0.0, 0.0, 0.0)},
            'RAnkleRoll':     {'axis': 'X', 'V': (0.0, 0.0, -0.04519)},

            
            # Fixed Torso->LShoulderPitch and LShoulderPitch->LShoulderRoll transforms are applied in the parent chain.
            'LShoulderPitch': {'axis': 'Y', 'V': (0.0, 0.0, 0.0)}, # No translation from Pitch to Roll axis
            'LShoulderRoll':  {'axis': 'Z', 'V': (0.105, 0.015, 0.0)}, # UpperArmLength(105mm) + ElbowOffsetY(15mm)
            'LElbowYaw':      {'axis': 'X', 'V': (0.0, 0.0, 0.0)}, # No translation from Yaw to Roll axis
            'LElbowRoll':     {'axis': 'Z', 'V': (0.05595, 0.0, 0.0)}, # LowerArmLength(55.95mm)
            
            # Right Arm
            'RShoulderPitch': {'axis': 'Y', 'V': (0.0, 0.0, 0.0)},
            'RShoulderRoll':  {'axis': 'Z', 'V': (0.105, -0.015, 0.0)}, # Note the negative Y offset for the right side
            'RElbowYaw':      {'axis': 'X', 'V': (0.0, 0.0, 0.0)},
            'RElbowRoll':     {'axis': 'Z', 'V': (0.05595, 0.0, 0.0)},
        }

     

        # T = R_axis(theta) * Trans(X, Y, Z)

        if joint_name == 'LHipYawPitch' or joint_name == 'RHipYawPitch':
            # These joints are coupled 45 degree rotations in the Y-Z plane.
            
            # Fixed translation V = (0, 0, 0) to the next joint (LHipRoll)
            X, Y, Z = 0.0, 0.0, 0.0
            
            # Pre-calculate common terms
            c_theta = np.cos(joint_angle)
            s_theta = np.sin(joint_angle)
            s_sqrt2 = s_theta / np.sqrt(2.0)
            c_sqrt2 = c_theta / np.sqrt(2.0)

            # Rotation Matrix R_YZ(theta) - Direct Formula
            T[0:3, 0:3] = np.array([
                [c_theta,  -s_sqrt2,                 s_sqrt2],
                [s_sqrt2,   c_theta - s_sqrt2,      -c_sqrt2 - s_sqrt2],
                [-s_sqrt2,  c_sqrt2 + s_sqrt2,       c_theta - s_sqrt2]
            ])

            # Translation part (T = R * Trans(V)). Since V = (0, 0, 0), the translation part is also (0, 0, 0)
            T[0, 3] = 0.0
            T[1, 3] = 0.0
            T[2, 3] = 0.0

            return T


        params = KINEMATIC_PARAMETERS [joint_name]
        axis = params['axis']
        X, Y, Z = params['V']

        c_theta = np.cos(joint_angle)
        s_theta = np.sin(joint_angle)

        if axis == 'X':
            # R_x rotation: [1, 0, 0], [0, c, -s], [0, s, c]
            T[0:3, 0:3] = np.array([
                [1.0, 0.0, 0.0],
                [0.0, c_theta, -s_theta],
                [0.0, s_theta, c_theta]
            ])
            T[0, 3] = X
            T[1, 3] = c_theta * Y - s_theta * Z
            T[2, 3] = s_theta * Y + c_theta * Z

        elif axis == 'Y':
            # R_y rotation: [c, 0, s], [0, 1, 0], [-s, 0, c]
            T[0:3, 0:3] = np.array([
                [c_theta, 0.0, s_theta],
                [0.0, 1.0, 0.0],
                [-s_theta, 0.0, c_theta]
            ])
            T[0, 3] = c_theta * X + s_theta * Z
            T[1, 3] = Y
            T[2, 3] = -s_theta * X + c_theta * Z

        elif axis == 'Z':
            # R_z rotation: [c, -s, 0], [s, c, 0], [0, 0, 1]
            T[0:3, 0:3] = np.array([
                [c_theta, -s_theta, 0.0],
                [s_theta, c_theta, 0.0],
                [0.0, 0.0, 1.0]
            ])
            T[0, 3] = c_theta * X - s_theta * Y
            T[1, 3] = s_theta * X + c_theta * Y
            T[2, 3] = Z

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        # print([j for j in joints])
        for chain_name, chain_joints in self.chains.items():
            # Initialize T with the fixed transform from Torso to the first joint (V_Torso)
            T = get_initial_transform(chain_name)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl 



                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
