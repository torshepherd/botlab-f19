#! /usr/bin/python
import lcm
from time import sleep
import sys
import math as np
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t



class WaypointFollower():
    def __init__(self, wplist):
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
        self.waypoints = wplist
        self.wpt_num = 1
        self.wpt_thresh = 0.1
        self.Kp = 1
        self.const_v = 0.2
   
    def odometry_handler(self, channel, data):
        msg = odometry_t().decode(data)
        goal = self.waypoints[self.wpt_num]
        dx = goal[0] - msg.x
        dy = goal[1] - msg.y
        goal_theta = np.atan2(dy,dx)
        dtheta = goal_theta - msg.theta
        if dtheta < -np.pi:
            dtheta += 2*np.pi
        if dtheta > np.pi:
            dtheta -= 2*np.pi
        self.p = np.sqrt((dx)**2 + (dy)**2)
        print("Current position: {}, {}, {}".format(msg.x, msg.y, msg.theta))
        print("   Goal position: {}, {}, {}".format(goal[0], goal[1], goal_theta))
        if self.p < self.wpt_thresh:
            self.wpt_num += 1
            self.wpt_num = self.wpt_num % 4
            goal = self.waypoints[self.wpt_num]
            dx = goal[0] - msg.x
            dy = goal[1] - msg.y
            self.p = np.sqrt((dx)**2 + (dy)**2)
        if abs(dtheta) > 0.2:
            self.v_cmd = 0
            if msg.theta < goal_theta:
                print("Turning left")
                self.w_cmd = 1.5
            else:
                print("Turning right")
                self.w_cmd = -1.5
        else:
            print("Driving")
            self.v_cmd = self.const_v
            self.w_cmd = self.Kp * (dtheta)


        self.motor_cmd_publish()

    def motor_cmd_publish(self):
        msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = self.v_cmd
        msg.angular_v = self.w_cmd
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

wps = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]]
W = WaypointFollower(wps)

while True:
    W.lc.handle()