'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import *


class StandingUpAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(StandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.initial_posture = None
        self.standing_up_motion = None

    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)
        
    
    def standing_up(self):
        posture = self.posture
        # YOUR CODE HERE
        
        self.last_posture = posture
        if  posture == "Back" or posture == "Belly":
            self.initial_posture = posture
        

        perception_to_action_back = {
            # "Belly": rightBackToStand,
            "Back": rightBackToStand,
            "Crouch": rightBackToStand,
            "HeadBack": rightBackToStand,
            "Left": rightBackToStand,
            "Right": rightBackToStand,
            "Sit": rightBackToStand,
            "Knee": rightBackToStand, 
            "Frog": rightBackToStand,
            "StandInit": rightBackToStand,
            "Stand": wipe_forehead,

        }


        perception_to_action_belly = {
            "Belly": leftBellyToStand,
            "Crouch": leftBellyToStand,
            "HeadBack": leftBellyToStand,
            "Left": leftBellyToStand,
            "Right": leftBellyToStand,
            "Sit": leftBellyToStand,
            "Knee": leftBellyToStand, 
            "Frog": leftBellyToStand,
            "StandInit": leftBellyToStand,
            "Stand": wipe_forehead,
        }
        

        keyframe_func = perception_to_action_back.get(posture)

        if self.initial_posture == "Belly":
            keyframe_func = perception_to_action_belly.get(posture)

        
        if keyframe_func is not None:
            print(f"Posture: {posture}, Executing keyframe motion", keyframe_func.__name__)
            self.keyframes = keyframe_func()
            
            



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