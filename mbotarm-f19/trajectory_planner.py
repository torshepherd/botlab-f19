import numpy as np
import time

"""
TODO: build a trajectory generator and waypoint planner
        so it allows your state machine to iterate through
        the plan.
"""

class TrajectoryPlanner():
    def __init__(self, mbot_arm):
        self.idle = True
        self.mbot_arm = mbot_arm
        self.num_joints = mbot_arm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints
        self.dt = 0.02 # command rate

    def set_initial_wp(self):
        self.initial_wp = self.mbot_arm.get_positions()

    def set_final_wp(self, waypoint):
        if(np.size(waypoint)<self.num_joints):
            print("ERROR length of waypoint vector < num_joints")
            return
        self.final_wp = waypoint

    def go(self, max_speed = 10.0):
        if(self.idle == True):
            delta_time = self.calc_time_from_waypoints(self.initial_wp,self.final_wp,max_speed)
            plan = self.generate_cubic_spline(self.initial_wp, self.final_wp, delta_time)
            self.execute_plan(plan)

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        max_delta = 0.0
        for i in range(self.num_joints):
            delta = abs(initial_wp[i] - final_wp[i])
            if(delta > max_delta):
                max_delta = delta
        time = 1.5 * max_delta / max_speed
        return time

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        num_steps = int(T/self.dt)+1
        T_mat = np.array([[1,0,0,0],[0,1,0,0],[1,T,pow(T,2),pow(T,3)],[0,1,2*T,3*pow(T,2)]])
        T_mat = np.linalg.inv(T_mat)
        J     = np.array([np.array(initial_wp),np.zeros(self.num_joints),np.array(final_wp),np.zeros(self.num_joints)])
        A     = np.dot(T_mat,J)
        plan  = np.zeros([num_steps,2*self.num_joints])

        for i in range(num_steps):
            for j in range(self.num_joints):
                t = i*self.dt
                pos   = A[0][j] + A[1][j]*t + A[2][j]*pow(t,2) + A[3][j]*pow(t,3)
                vel   = A[1][j] + 2*A[2][j]*t + 3*A[3][j]*pow(t,2)
                plan[i][j] = pos
                plan[i][j+self.num_joints] = vel
        #print(plan)
        return plan

    def execute_plan(self, plan, look_ahead=8):
        self.idle = False
        for i in range(np.shape(plan)[0]):
            j = np.clip(i+look_ahead, 0, np.shape(plan)[0]-1)
            if(self.mbot_arm.estop == False):
                self.mbot_arm.set_speeds(plan[i][5:10])
                self.mbot_arm.set_positions(plan[j][0:5])
                self.mbot_arm.get_feedback()
                time.sleep(self.dt)
	    else:
		break
        self.idle = True
