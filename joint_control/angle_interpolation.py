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
        self.motion_is_complete = False


    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
    
        if "LHipYawPitch" in target_joints.keys():
            target_joints['RHipYawPitch'] = target_joints["LHipYawPitch"] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names, times, keys = keyframes
        now_time = perception.time
        
        # If this is the first call, set a start time offset
        if not hasattr(self, 'motion_start_time'):
            self.motion_start_time = now_time
        
        # Calculate time relative to when the motion started
        elapsed_time = now_time - self.motion_start_time


        max_duration = 0.0
        for joint_times in times:
            if joint_times:
                max_duration = max(max_duration, joint_times[-1])
    
        self.motion_is_complete = (elapsed_time >= max_duration)
        
        # print(f"Absolute time: {now_time:.2f}, Elapsed motion time: {elapsed_time:.2f}")  # Debug
        
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
                target_joints[joint_name] = joint_keys[-1][0]  # Hold last position
                # Or to loop: self.motion_start_time = now_time (uncomment to loop)

                # self.motion_start_time = now_time  # Reset start time to loop
                # target_joints[joint_name] = joint_keys[0][0]  # Use first frame
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
                        

                       
                        
                        # Linear interpolation
                        # target_angle = angle0 + (angle1 - angle0) * t_norm

                        target_angle = cubic_bezier(t_norm, P0, P1, P2, P3)
                        target_joints[joint_name] = target_angle
                        
                        # Debug
                        # if joint_name == "HeadPitch":
                        #     print(f"HeadPitch: time {elapsed_time:.2f} in segment {i} ({t0:.2f}-{t1:.2f}), angle: {target_angle:.3f}")
                        break
        
        return target_joints
    
if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = keyframes.rightBackToStand()
    agent.run()
