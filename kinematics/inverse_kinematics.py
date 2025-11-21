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
import numpy as np




def get_initial_transform(chain_name):
    '''Fixed translation from Torso to the first joint of the chain.'''
    TORSO_TRANSFORMS = {
        'Head': (0.0, 0.0, 0.1265),
        'LArm': (0.0, 0.098, 0.100),
        'RArm': (0.0, -0.098, 0.100),
        'LLeg': (0.0, 0.050, -0.085), # Torso -> LHipYawPitch
        'RLeg': (0.0, -0.050, -0.085) # Torso -> RHipYawPitch
    }
    if chain_name not in TORSO_TRANSFORMS:
        return identity(4) 
        
    X, Y, Z = TORSO_TRANSFORMS[chain_name]
    T = identity(4)
    T[0, 3] = X
    T[1, 3] = Y
    T[2, 3] = Z
    return T


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        L_Thigh = 0.100       
        L_Tibia = 0.1029      
        L_Foot_Z = 0.04519    
        
        T_Torso_to_Start = get_initial_transform(effector_name)
        T_Start_to_End = np.linalg.inv(T_Torso_to_Start) @ transform

        P_AnkleRoll = T_Start_to_End[0:3, 3]
        R_AnkleRoll = T_Start_to_End[0:3, 0:3]
        
        Z_target = R_AnkleRoll[:, 2] 
        P_AnklePitch = P_AnkleRoll - L_Foot_Z * Z_target
        
        X, Y, Z = P_AnklePitch[0], P_AnklePitch[1], P_AnklePitch[2]
        
        D_squared = X**2 + Y**2 + Z**2
        D = np.sqrt(D_squared)

        if D > (L_Thigh + L_Tibia) or D < np.abs(L_Thigh - L_Tibia):
             print(f"IK Warning: Target for {effector_name} is unreachable. Distance D={D:.4f}m.")
             return [0.0] * 6 

        cos_gamma = (L_Thigh**2 + L_Tibia**2 - D_squared) / (2 * L_Thigh * L_Tibia)
        gamma = np.arccos(np.clip(cos_gamma, -1, 1))
        
        theta4 = -(np.pi - gamma) 

        theta6 = np.arctan2(R_AnkleRoll[0, 1], R_AnkleRoll[1, 1])

        angle_D_Z_plane = np.arctan2(X, -Z)
        
        beta = np.arccos(np.clip((L_Thigh**2 + D_squared - L_Tibia**2) / (2 * L_Thigh * D), -1, 1))

        theta3 = angle_D_Z_plane + beta
        
        theta5 = np.arctan2(R_AnkleRoll[2, 0], R_AnkleRoll[2, 2]) - theta3 - theta4 

        theta1 = 0.0 
        theta2 = 0.0 
        
        joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE

        chain_name = 'L' + effector_name[1:] if effector_name.startswith('LL') else 'R' + effector_name[1:]
        
        joint_angles = self.inverse_kinematics(chain_name, transform)
        
        if len(joint_angles) == 6:
            joint_names = self.chains.get(chain_name, [])
            
            self.keyframes = (
                joint_names,       
                joint_angles,      
                [1.0] * 6          
            )
            print(f"IK solved for {effector_name}. Target Angles (rad): {[f'{a:.3f}' for a in joint_angles]}")
        else:
            self.keyframes = ([], [], [])
            print(f"IK solution failed or returned incomplete angles for {effector_name}. Movement aborted.")

        # self.keyframes = ([], [], [])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
