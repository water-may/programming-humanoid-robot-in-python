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
        self.standing_up_motion = False
        self.initial_posture = None
        self.initial_phase_complete = False

    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)
        
    
    def standing_up(self):
        posture = self.posture



        #TODO

        # step one, if the robot is BACK or BELLY, set the standing up flag to true and start standing up;
        # set initial posture to one of the above
        # if the standing_up is not complete but it is Back or Belly restart the motion 
        # when the posture is stand then stop

        # if the robot has fallen down after starting up then start over again 
        if posture == "Back" or posture == "Belly":
            if self.standing_up_motion == True and self.initial_phase_complete:
                print("Robot has fallen, resettting motion!")
                self.reset_motion_timing()
                self.standing_up_motion = False
                self.initial_phase_complete = False


        if posture != self.initial_posture:
            self.initial_phase_complete = True


        # case where robot has stood up 
        if posture == "Stand" and self.standing_up_motion:
            self.standing_up_motion = False
            self.initial_posture = None
            return 
        
        # if the robot is idle and laying down then start the motion 
        if not self.standing_up_motion and (posture == "Back" or posture == "Belly"):
            self.initial_posture = posture
            self.standing_up_motion = True
            



        if self.standing_up_motion:
            if self.initial_posture == "Back":
                self.keyframes = rightBackToStand()
            elif self.initial_posture == "Belly":
                self.keyframes = leftBellyToStand()


        print("Init Posture: ", self.initial_posture)
        # print("keyframe: ", self.keyframes )
        
            



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