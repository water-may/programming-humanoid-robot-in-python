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
        if posture == "Stand":
            self.motion_is_complete = True
            self.last_posture = "Stand"  
            return
            
        if hasattr(self, 'last_posture') and self.last_posture == "Stand" and posture != "Stand":
            self.motion_is_complete = False
            print(f"Fallen from standing! Current posture: {posture}")
        
        self.last_posture = posture

        if self.motion_is_complete:
            return
        
        if  posture == "Back" | posture == "Belly":
            self.initial_posture = posture
        

        perception_to_action_back = {
            "Belly": rightBellyToStand,
            "Back": rightBackToStand,
            "Crouch": rightBackToStand,
            "HeadBack": rightBackToStand,
            "Left": rightBackToStand,
            "Right": rightBackToStand,
            "Sit": rightBackToStand,
            "Knee": rightBackToStand, 
            "Frog": rightBackToStand,
        }


        perception_to_action_belly = {
            "Belly": rightBellyToStand,
            "Back": rightBackToStand,
            "Crouch": rightBellyToStand,
            "HeadBack": rightBellyToStand,
            "Left": rightBellyToStand,
            "Right": rightBellyToStand,
            "Sit": rightBellyToStand,
            "Knee": rightBellyToStand, 
            "Frog": rightBellyToStand,
        }
        

        keyframe_func = perception_to_action_belly.get(posture)

        if self.initial_posture == "Belly":
            keyframe_func = perception_to_action_belly.get(posture)

        
        if keyframe_func is not None:
            print(f"Posture: {posture}, Executing keyframe motion")
            self.keyframes = keyframe_func()
            self.motion_is_complete = False
            
            



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
        self.is_moving = False


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