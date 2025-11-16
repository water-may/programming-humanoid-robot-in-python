'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


import math
from pid import PIDAgent
import keyframes 


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])


    def think(self, perception):

        
        target_joints = self.angle_interpolation(self.keyframes, perception)
    
        if target_joints and "LHipYawPitch" in target_joints.keys():
            print("KEY available")
            target_joints['RHipYawPitch'] = target_joints["LHipYawPitch"] # copy missing joint in keyframes
            self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)


    def reset_motion_timing(self):
        """Call this to reset/stop the current motion and start fresh"""
        self.reset_motion = True
        self.motion_finished = False


    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names, times, keys = keyframes
        now_time = perception.time
        
        # Check if we should reset/stop the current motion
        if hasattr(self, 'reset_motion') and self.reset_motion:
            self.current_keyframes = keyframes
            self.motion_start_time = now_time
            self.motion_finished = False
            self.reset_motion = False  # Reset the flag
    
        # Reset motion start time when keyframes change
        if not hasattr(self, 'current_keyframes') or self.current_keyframes != keyframes:
            self.current_keyframes = keyframes
            self.motion_start_time = now_time
            self.motion_finished = False
        
        # Calculate time relative to when the motion started
        elapsed_time = now_time - self.motion_start_time


         # Check if all motions are finished
        max_motion_time = 0
        for joint_times in times:
            if joint_times and joint_times[-1] > max_motion_time:
                max_motion_time = joint_times[-1]
        
        if elapsed_time >= max_motion_time:
            self.motion_finished = True
            # Hold the final position
            for joint_index, joint_name in enumerate(names):
                if keys[joint_index]:
                    target_joints[joint_name] = keys[joint_index][-1][0]
            return target_joints
        

        def cubic_bezier(t, p0, p1, p2, p3):
            """Cubic Bezier interpolation"""
            u = 1 - t
            return (u*u*u * p0 + 
                    3 * u*u * t * p1 + 
                    3 * u * t*t * p2 + 
                    t*t*t * p3)
    

        for joint_index, joint_name in enumerate(names):
            joint_times = times[joint_index]
            joint_keys = keys[joint_index]
            
            if len(joint_times) == 0:
                continue

            # Use elapsed_time instead of absolute time
            if elapsed_time < joint_times[0]:
                # Before first keyframe
                target_joints[joint_name] = joint_keys[0][0]
            elif elapsed_time >= joint_times[-1]:
                # After last keyframe - loop or hold last position
                target_joints[joint_name] = joint_keys[-1][0]
                continue  
            else:
                # Find which segment we're in
                for i in range(len(joint_times) - 1):
                    if joint_times[i] <= elapsed_time < joint_times[i + 1]:
                        # Simple linear interpolation
                        t0 = joint_times[i]
                        t1 = joint_times[i + 1]


                        t_norm = (elapsed_time - t0) / (t1 - t0)

                        start_key = joint_keys[i]
                        end_key = joint_keys[i + 1]

                        angle0 = start_key[0]
                        angle1 = end_key[0]

                        out_handle = start_key[2]
                        in_handle = end_key[1]

                        P0 = angle0
                        P3 = angle1
                        P1 = angle0 + out_handle[2]
                        P2 = angle1 + in_handle[2]
                        

                        target_angle = cubic_bezier(t_norm, P0, P1, P2, P3)
                        target_joints[joint_name] = target_angle
                        
                        break
        
        return target_joints
    
if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = keyframes.rightBellyToStand()
    agent.run()
