'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello, leftBackToStand
import pickle
import numpy as np

ROBOT_POSE_CLF ='robot_pose.pkl'
FEATURE_NAMES = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
                 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',
                 'AngleX', 'AngleY']

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier =  pickle.load(open(ROBOT_POSE_CLF, 'rb')) # LOAD YOUR CLASSIFIER
        self.classes = ['Knee', 'Left', 'Back', 'Right', 'Belly', 'Frog', 'Sit', 'StandInit', 'Crouch', 'Stand', 'HeadBack']

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        # print("perception: ", perception)
        

        if self.posture_classifier is None:
            return posture

        current_features = []
        
        
        joint_names = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 
                       'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        
        for name in joint_names:
            
            current_features.append(perception.joint.get(name, 0.0))

        try:
            current_features.append(perception.imu[0]) # AngleX
            current_features.append(perception.imu[1]) # AngleY
        except (IndexError, AttributeError):
            # Fallback if IMU data isn't initialized or is incomplete
            current_features.extend([0.0, 0.0])

        if len(current_features) != 10:
             print("Warning: Missing feature data!")
             return 'unknown' # Don't predict with bad data
        
        X_predict = np.array(current_features).reshape(1, -1)

        predicted_index = self.posture_classifier.predict(X_predict)[0]
        posture = self.classes[predicted_index]
        # print(posture)
        
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
