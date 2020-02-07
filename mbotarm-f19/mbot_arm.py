#mot_arm.py
import numpy as np
import time
import math

"""
TODO:

Implement the missing functions
add anything you see fit

"""

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
PI = np.pi

def clamp_radians(theta):
    return np.clip(theta, -np.pi, np.pi)

class MBotArm():
    def __init__(self, joints):
        self.joints = joints
        self.gripper = joints[-1]
        self.initial_pos = np.array([108,-110,55,130,0],dtype=np.float)*D2R
        self.gripper_open_pos = np.deg2rad(-90.0)
        self.gripper_closed_pos = np.deg2rad(8.0)
        self.gripper_state = True
        self.estop = False
        """TODO: Find the physical angle limits of the mbot arm. Remember to keep track of this if you include more motors"""
        self.angle_limits = np.array([
                            [-150, -105, -120, -130, -150],
                            [ 150,  105,  120,  130, 150]], dtype=np.float)*D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1
        self.load_fb = [0.0] * self.num_joints         # -1 to 1
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        """ Arm Lengths """
        # TODO: Fill in the measured dimensions.
        self.base_len     = 56.0
        self.shoulder_len = 54.0
        self.elbow_len    = 54.0
        self.wrist_len    = 112.0

    def initialize(self):
        for index, joint in enumerate(self.joints):
            joint.enable_torque()
            joint.set_position(self.initial_pos[index])
            joint.set_torque_limit(0.95)
            joint.set_speed(0.95)

    def open_gripper(self):
        """ TODO """
	self.gripper.set_position(self.gripper_open_pos)
        self.gripper_state = False
        pass

    def close_gripper(self):
        """ TODO """
	self.gripper.set_position(self.gripper_closed_pos)
        self.gripper_state = True
        pass

    def set_positions(self, joint_angles, update_now = True):
        joint_angles = self.clamp(joint_angles)
        for i,joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if(update_now):
                joint.set_position(joint_angles[i])

    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i])

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg)

    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i])

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque()

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque()

    def get_positions(self):
        for i,joint in enumerate(self.joints):
            pos = joint.get_position()
            if(pos != None):
                self.joint_angles_fb[i] = pos
        #print(self.joint_angles_fb)
        return self.joint_angles_fb

    def get_speeds(self):
        for i,joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed()
        return self.speed_fb

    def get_loads(self):
        for i,joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load()
        return self.load_fb

    def get_temps(self):
        for i,joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp()
        return self.temp_fb

    def get_moving_status(self):
        for i,joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving()
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        for i, joint in enumerate(joint_angles):
            if joint > self.angle_limits[1][i]:
                joint_angles[i] = self.angle_limits[1][i]
            if joint < self.angle_limits[0][i]:
                joint_angles[i] = self.angle_limits[0][i]
        return joint_angles

    def get_fb_pose(self):
        pose = self.get_pose(self.joint_angles_fb)
        debugIK = False
        if(debugIK):
            if(self.joint_angles_fb[1]!=0):
                print("Actual:")
                print(self.joint_angles_fb)
                print("IK:")
                if(self.joint_angles_fb[2]<0):
                    print(self.IK(pose,0)[0])
                if(self.joint_angles_fb[2]>0):
                    print(self.IK(pose,1)[0])
        return self.get_pose(self.joint_angles_fb)

    def get_pose(self, joint_angles):
        dh_table = np.array([[joint_angles[0], self.base_len, 0, PI/2],
                             [joint_angles[1] + PI/2, 0, self.shoulder_len, 0],
                             [joint_angles[2], 0, self.elbow_len, 0],
                             [joint_angles[3], 0, self.wrist_len, 0]])
        return self.FK(dh_table, 4)

    def FK(self, dh_table, link):
        # base transformation
        T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        phi = 0
        for i in range(link):
            Rem_T = [[math.cos(dh_table[i,0]), -math.sin(dh_table[i,0]) * math.cos(dh_table[i,3]), math.sin(dh_table[i,0]) * math.sin(dh_table[i,3]), math.cos(dh_table[i,0]) * dh_table[i,2]],
                     [math.sin(dh_table[i,0]), math.cos(dh_table[i,0]) * math.cos(dh_table[i,3]), -math.cos(dh_table[i,0]) * math.sin(dh_table[i,3]), math.sin(dh_table[i,0]) * dh_table[i,2]],
                     [0, math.sin(dh_table[i,3]), math.cos(dh_table[i,3]), dh_table[i,1]],
                     [0, 0, 0, 1]]
            T = np.matmul(T, Rem_T)
            if i > 0:
                phi = phi + dh_table[i,0]
        x = T[0,3]
        y = T[1,3]
        z = T[2,3]
        return (x, y, z, phi)

    def IK(self, pose, cfg):
        ### These IK only work for positions in positive x ###
        ### TODO: Add degenerate case for negative x ###
        x = pose[0]
        y = pose[1]
        z = pose[2]
        phi = pose[3]
        r = math.sqrt(x**2 + y**2)
        if r<140:
            phi = -1
            z += 10
        l1 = self.shoulder_len
        l2 = self.elbow_len
        base = self.base_len
        ef = self.wrist_len
        theta = [0,0,0,0]
        if(x>0):
            theta[0] = math.atan2(y, x)
        if(x<0):
            theta[0] = math.atan2(y, -x)
        S = z - base - ef * math.sin(phi)
        R = math.sqrt(x**2 + y**2) - ef * math.cos(phi)
        D = (R**2 + S**2 - l1**2 - l2**2)/(2 * l2 * l1)
        if 1-D**2 < 0:
            return theta,False
        if(x>0):
            #elbow up
            if cfg == 0:
                theta[2] = math.atan2(-math.sqrt(1-D**2),D)
                theta[1] = math.atan2(S,R)-math.atan2(l2*math.sin(theta[2]),l1+l2*math.cos(theta[2]))-math.pi/2.0
            #elbow down
            elif cfg == 1:
                theta[2] = math.atan2(math.sqrt(1-D**2),D)
                theta[1] = math.atan2(S,R)-math.atan2(l2*math.sin(theta[2]),l1+l2*math.cos(theta[2]))-math.pi/2.0
            theta[3] = phi - theta[1] - theta[2] - math.pi/2.0
        if(x<0):
            x = -x
            #elbow up
            if cfg == 0:
                theta[2] = math.atan2(-math.sqrt(1-D**2),D)
                theta[1] = math.atan2(S,R)-math.atan2(l2*math.sin(theta[2]),l1+l2*math.cos(theta[2]))-math.pi/2.0
            #elbow down
            elif cfg == 1:
                theta[2] = math.atan2(math.sqrt(1-D**2),D)
                theta[1] = math.atan2(S,R)-math.atan2(l2*math.sin(theta[2]),l1+l2*math.cos(theta[2]))-math.pi/2.0
            theta[3] = phi - theta[1] - theta[2] - math.pi/2.0
            theta = [a*-1 for a in theta]
            while theta[3] < -np.pi/2:
                theta[3] += np.pi
        return theta,True
