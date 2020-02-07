import time
import numpy as np
import cv2
import math

import lcm
import os
os.sys.path.append('lcmtypes/')
# Messages used for communication with Mbot programs.
# TODO: Add your customized lcm messages as needed.
from lcmtypes import pose_xyt_t
from lcmtypes import mbot_arm_block_t
from lcmtypes import mbot_arm_block_list_t
from lcmtypes import mbot_arm_cmd_t
from lcmtypes import mbot_arm_status_t
from lcmtypes import mbot_picking_status_t
from lcmtypes import block_request_t
from lcmtypes import robot_path_t
from lcmtypes import path_status_t

D2R = 3.141592/180.0
R2D = 180.0/3.141592
#subscribe
ARM_STATE_CH = "BOT_ARM_STATUS"
BOT_BLOCK_STATUS_CH = "BOT_BLOCK_STATUS_CH"
NEXT_BLOCK_REQUEST_CH = "NEXT_BLOCK_REQUEST_CH"
SLAM_POSE_CH = "SLAM_POSE"
PATH_STATUS_CHANNEL = "PATH_COMPLETE_STATUS"
#publish
ARM_COMMAND_CH = "BOT_ARM_COMMAND"
ARM_BLOCK_STATUS_CH = "ARM_BLOCK_STATUS_CH"
NEXT_BLOCK_RETURN_CH = "NEXT_BLOCK_RETURN_CH"
CONTROLLER_PATH_CHANNEL = "CONTROLLER_PATH"

THETA_THRESH = 0.15

DETECT_NONE = 0
DETECT_BLOCKS = 1
DETECT_BLOCK_IN_RANGE = 2
ATTEMPT_GRASP = 3

"""
TODO: Add states and state functions to this class
        to implement all of the required logics
"""
def sortBlock(tag):
    return tag.pose_t[2]

class StateMachine():
    def __init__(self, mbot_arm, planner):
        self.mbot_arm = mbot_arm
        self.tp = planner
        self.tags = []
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.lc = lcm.LCM()
        self.camera_extrinsics = np.zeros((4,4),dtype=float)

        self.subscription = self.lc.subscribe(ARM_COMMAND_CH, self.mbot_arm_cmd_handler)
        self.subscription = self.lc.subscribe(NEXT_BLOCK_REQUEST_CH, self.block_request_handler)
        self.subscription = self.lc.subscribe(BOT_BLOCK_STATUS_CH, self.mbot_block_status_handler)
        self.subscription = self.lc.subscribe(SLAM_POSE_CH, self.slam_pose_handler)
        self.subscription = self.lc.subscribe(PATH_STATUS_CHANNEL, self.path_complete_handler)
        #for move towards and grabbing
        self.arm_status = mbot_arm_status_t()
        self.arm_block = mbot_arm_block_t()
        #for interaction with explorer
        self.arm_block_list = []#mbot_arm_block_t()
        self.arm_block_ids = []#int
        self.cur_block = mbot_arm_block_t()#keep track of current block (move towards to)
        self.mbot_picking_status = mbot_picking_status_t()#update together with explorer
        self.cur_pose = pose_xyt_t()#subscribe to slam pose
        #initialization for block searching task
        self.robot_path = robot_path_t()
        self.robot_path.path = [pose_xyt_t(),pose_xyt_t()]
        self.robot_path.path_length = 2
        self.robot_path.path_type = 5
        self.path_completed = True
        self.complete_counter = -1
        self.new_image = False

        self.initialized = 0
        self.block_held = 0
        self.block_counter = -1
        self.cur_block.tag_id = -1
        self.mbot_picking_status.status = self.mbot_picking_status.READY_FOR_NEXT

        self.cur_pose.x = 0
        self.cur_pose.y = 0
        self.cur_pose.theta = 0
        self.finished_tag_ids = []

        self.block_pre_state = DETECT_NONE
        self.block_cur_state = DETECT_NONE

        self.lost_tag_count = 0
        self.lost_tag_max_count = 50

    def set_current_state(self, state):
        self.current_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            self.manual()

        if(self.current_state == "idle"):
            self.idle()

        if(self.current_state == "estop"):
            self.estop()

        if(self.current_state == "calibrate_extrinsics"):
            self.calibrate_extrinsics()

        if(self.current_state == "detect_tags"):
            self.detect_tags()

        if(self.current_state == "preparing_grab"):
            self.preparing_grab()

        if (self.current_state == "stop_detection"):
            self.stop_detection()

        if(self.current_state == "grab_block"):
            self.grab_block()

        if(self.current_state == "release_block"):
            self.release_block()

        self.lc.handle_timeout(10)
        self.mbot_arm.get_feedback()


    """Functions run for each state"""
    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.mbot_arm.send_commands()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check MBot arm and restart program"
        self.mbot_arm.disable_torque()


    def detect_tags(self,verbose = True):
        print('detecting tags')
        if (verbose):
            print('id size:')
            print(self.arm_block_ids)
            print('block size:')
            print(len(self.arm_block_list))
        self.set_current_state("detect_tags")
        for haha in self.arm_block_list:
            print("detect = ", haha.tag_id, haha.pose.x)

        if len(self.tags) == 0:
            self.block_cur_state = DETECT_NONE

        self.tags.sort(key = sortBlock)
        if len(self.tags) > 0:
            for tag in self.tags:
                print('tag id:',tag.tag_id)
                print('x:',tag.pose_t[2])

                temp_block = mbot_arm_block_t()
                temp_block.tag_id = tag.tag_id
                # x = tag.pose_t[2]*1000+40
                # y = tag.pose_t[0]*1000+13
                # if (verbose):
                #     print('detecting block now:')
                #     print(x)
                #     print(y)

                temp_block.pose.x = self.cur_pose.x
                temp_block.pose.y = self.cur_pose.y
                temp_block.pose.theta = self.cur_pose.theta

                #initial stage
                if not self.initialized:
                    if (tag.tag_id not in self.arm_block_ids) and (tag.tag_id not in self.finished_tag_ids):
                        self.arm_block_ids.append(tag.tag_id)
                        self.arm_block_list.append(temp_block)
                        #sort according to pose.x
                        #self.arm_block_list.sort(key = sortBlock)
                else:
                    if (tag.tag_id not in self.finished_tag_ids):
                        if (tag.tag_id not in self.arm_block_ids):
                            #dont have any target block now, go grab it
                            if (self.cur_block.tag_id == -1):
                                print("if 1")
                                self.cur_block = temp_block
                                self.mbot_picking_status.status = self.mbot_picking_status.PICKING_UP
                                for i in range(0,3):
                                    self.lc.publish(ARM_BLOCK_STATUS_CH, self.mbot_picking_status.encode())
                                    time.sleep(0.02)

                                self.set_current_state("preparing_grab")
                                break
                            #have target block, put it in list first
                            else:
                                print("else 1")
                                self.arm_block_ids.append(tag.tag_id)
                                self.arm_block_list.append(temp_block)
                                #sort according to pose.x
                                #self.arm_block_list.sort(key = sortBlock)
                        else:
                            for j in range(len(self.arm_block_list)):
                                if self.arm_block_list[j].tag_id == tag.tag_id:
                                    print("update loc")
                                    self.arm_block_list[j].pose.x = self.cur_pose.x
                                    self.arm_block_list[j].pose.y = self.cur_pose.y
                                    self.arm_block_list[j].pose.theta = self.cur_pose.theta

                    if tag.tag_id in self.arm_block_ids:
                        #found our target block
                        if self.cur_block.tag_id == tag.tag_id:
                            print("if 2")
                            self.mbot_picking_status.status = self.mbot_picking_status.PICKING_UP
                            for i in range(0,3):
                                self.lc.publish(ARM_BLOCK_STATUS_CH, self.mbot_picking_status.encode())
                                time.sleep(0.02)
                            self.set_current_state("preparing_grab")
                            break
                        #in the list, not our target block
                        else:
                            print("else 2")
                            pass
                            # if self.cur_block.tag_id != -1:
                            #     self.arm_block_ids.append(cur_block.tag_id)
                            #     self.arm_block_list.append(cur_block)
                            #     cur_block = temp_id

    #responsible for move towards block when detecting it
    def preparing_grab(self):
        print('preparing_grab')
        self.set_current_state("preparing_grab")
        
        if self.new_image == True:
            print("new image")
            if not self.path_completed:
                print("completed")
                pass
            else:
                if len(self.tags) == 0:
                    self.robot_path_gen(self.cur_block.pose.x,self.cur_block.pose.y,self.cur_block.pose.theta, False)
                    self.lost_tag_count += 1
                    if (self.lost_tag_count <= self.lost_tag_max_count):
                        time.sleep(0.1)
                        return
                    print('lost tag 1')
                    for m in range(0,2):
                        self.lc.publish(CONTROLLER_PATH_CHANNEL, self.robot_path.encode())
                    while not self.path_completed:
                        time.sleep(0.1)
                else:
                    self.lost_tag_count = 0
                flag_get_tag = False
                if len(self.tags) > 0:
                    print('moving towards it')
                    #find our target tag
                    i = 0
                    while (i < len(self.tags) ):
                        if( self.tags[i].tag_id == self.cur_block.tag_id) :                            
                            flag_get_tag = True
                            break
                        i+=1
                if not flag_get_tag:
                    self.lost_tag_count += 1
                    if (self.lost_tag_count <= self.lost_tag_max_count):
                        time.sleep(0.1)
                        return
                    print('lost tag 2')
                    self.robot_path_gen(self.cur_block.pose.x,self.cur_block.pose.y,self.cur_block.pose.theta, False)
                    for m in range(0,2):
                        self.lc.publish(CONTROLLER_PATH_CHANNEL, self.robot_path.encode())
                    while not self.path_completed:
                        time.sleep(0.1)
                else:
                    self.lost_tag_count = 0
                    self.arm_status.utime = time.time()

                    self.arm_block.tag_id = self.tags[i].tag_id
                    x = self.tags[i].pose_t[2]*1000+40
                    y = -self.tags[i].pose_t[0]*1000+13
                    r = math.sqrt(x**2 + y**2)
                    print('location:')
                    print(x)
                    print(y)
                    print(r)

                    if self.mbot_arm.IK(np.array([x,y,-0.033*1000+30,-0.5]),0)[1] == True and r<200:
                        print('ready to grap')
                        self.set_current_state("grab_block")
                    else:
                        x_p = 0.0
                        y_p = 0.0
                        theta_p = 0.0
                        theta = -math.atan2(y, x+150)
                        if self.tags[i].center[0]>852:
                            print('pixel:')
                            print(self.tags[i].center[0])
                            print('rotating to right')
                            theta_p = -0.08
                        elif self.tags[i].center[0]<360:
                            print('pixel:')
                            print(self.tags[i].center[0])
                            print('rotating to left')
                            theta_p = 0.08
                        else:
                            print('translating')
                            x_p = 0.05
                        self.robot_path_gen(x_p,y_p,theta_p, True)
                        self.path_completed = False
                        self.new_image = False
                        for m in range(0,3):
                            self.lc.publish(CONTROLLER_PATH_CHANNEL, self.robot_path.encode())


        #     print('detecting block now:')
        #     print(x)
        #     print(y)
        #     print(r)
        #     if r>200:#mm
        #         self.arm_status.status = DETECT_BLOCKS
        #     else:
        #         self.arm_status.status = DETECT_BLOCK_IN_RANGE
        #     self.arm_block.pose.x = x*0.001
        #     self.arm_block.pose.y = y*0.001
        #     self.arm_block.pose.theta = -math.atan2(y, x+100)+np.pi/2
        #     self.arm_status.block = self.arm_block
        #
        #     #update block status
        #     self.block_cur_state = self.arm_status.status
        #
        #     #only publish when there is a change in block status
        #     if self.block_cur_state != self.block_pre_state:
        #         self.lc.publish(ARM_STATE_CH, self.arm_status.encode())
        #         print(self.arm_status.status)
        #
        # # Set the next state
        # self.block_pre_state = self.block_cur_state
        # print('cur_state:')
        # print(self.block_cur_state)
        # if self.block_cur_state == DETECT_BLOCK_IN_RANGE:
        #     self.set_current_state("grab_block")
        # else:
        #     self.set_current_state("preparing_grab")


    def stop_detection(self):
        self.set_current_state("idle")


    def grab_block(self):
        print('grab_block')
        i = 0
        pose_above = np.array([0,0,0,-0.5])
        if (len(self.tags) > 0):
            #find our target tag
            while (self.tags[i].tag_id != self.cur_block.tag_id) and (i<len(self.tags)):
                i+=1
            pose_above[0] = self.tags[i].pose_t[2]*1000+40 #4 cm offset along x
            pose_above[1] = -self.tags[i].pose_t[0]*1000+13 #1.3 cm offset along y
            pose_above[2] = -0.033*1000+30 #self.tags[0].pose_t[1]
        pose_grab = pose_above
        pose_grab[2] -= 30
        self.mbot_arm.open_gripper()
        cfg_standby_open = np.append(self.mbot_arm.initial_pos[:-1],self.mbot_arm.gripper_open_pos)
        cfg_above_open = np.append(self.mbot_arm.IK(pose_above,0)[0],self.mbot_arm.gripper_open_pos)
        cfg_grab_open = np.append(self.mbot_arm.IK(pose_grab,0)[0],self.mbot_arm.gripper_open_pos)
        cfg_standby_closed = np.append(self.mbot_arm.initial_pos[:-1],self.mbot_arm.gripper_closed_pos)
        cfg_above_closed = np.append(self.mbot_arm.IK(pose_above,0)[0],self.mbot_arm.gripper_closed_pos)
        cfg_grab_closed = np.append(self.mbot_arm.IK(pose_grab,0)[0],self.mbot_arm.gripper_closed_pos)
        waypoints = np.array([cfg_standby_open, cfg_above_open, cfg_grab_open, cfg_grab_closed, cfg_above_closed, cfg_standby_closed])

        for wp in waypoints:
            if(self.next_state == "estop"):
                break
            goal_wp = wp
            self.tp.set_initial_wp()
            self.tp.set_final_wp(goal_wp)
            self.tp.go(max_speed=10.0)
        #update status
        self.block_held = 1
        for i in range(0,5):
            self.mbot_picking_status.status = self.mbot_picking_status.RETURN_HOME
            self.lc.publish(ARM_BLOCK_STATUS_CH, self.mbot_picking_status.encode())
            time.sleep(0.02)
            
        self.set_current_state("idle")


    def release_block(self):
        print('release_block')
        pose_above = np.array([150.0,10.0,10,-0.5])
        pose_release = np.array([150.0,10.0,-5,-0.5])
        # if len(self.tags) > 0:
        #     pose_release[2] -= 30
        self.mbot_arm.open_gripper()
        cfg_standby_closed = np.append(self.mbot_arm.initial_pos[:-1],self.mbot_arm.gripper_closed_pos)
        cfg_above_closed = np.append(self.mbot_arm.IK(pose_above,0)[0],self.mbot_arm.gripper_closed_pos)
        cfg_release_closed = np.append(self.mbot_arm.IK(pose_release,0)[0],self.mbot_arm.gripper_closed_pos)
        cfg_standby_closed = np.append(self.mbot_arm.initial_pos[:-1],self.mbot_arm.gripper_closed_pos)
        cfg_above_open = np.append(self.mbot_arm.IK(pose_above,0)[0],self.mbot_arm.gripper_open_pos)
        cfg_release_open = np.append(self.mbot_arm.IK(pose_release,0)[0],self.mbot_arm.gripper_open_pos)
        waypoints = np.array([cfg_standby_closed, cfg_above_closed, cfg_release_closed, cfg_release_open, cfg_above_open, cfg_standby_closed])

        for wp in waypoints:
            if(self.next_state == "estop"):
                break
            goal_wp = wp
            self.tp.set_initial_wp()
            self.tp.set_final_wp(goal_wp)
            self.tp.go(max_speed=10.0)
        #update status
        self.finished_tag_ids.append(self.cur_block.tag_id)

        self.cur_block.tag_id = -1
        self.block_held = 0
        self.mbot_picking_status.status = self.mbot_picking_status.READY_FOR_NEXT
        for i in range(0,3):
            self.lc.publish(ARM_BLOCK_STATUS_CH, self.mbot_picking_status.encode())
            time.sleep(0.02)

        self.set_current_state("detect_tags")

    def mbot_arm_cmd_handler(self, channel, data):
        """
        Feedback Handler for mbot status
        this is run when a feedback message is recieved
        """
        print('handler working')
        msg = mbot_arm_status_t.decode(data)
        if (msg.status == ATTEMPT_GRASP):
            self.set_current_state("preparing_grab")

    def mbot_block_status_handler(self, channel, data):
        """
        Feedback Handler for mbot status
        this is run when a feedback message is recieved
        """
        print('arm picking status handler working')
        msg = mbot_picking_status_t.decode(data)
        self.mbot_picking_status.status = msg.status
        if (msg.status == msg.PUTTING_DOWN):
            self.set_current_state("release_block")
        if (msg.status == msg.DRIVE_TO_POSE):
            self.set_current_state("detect_tags")

    def block_request_handler(self, channel, data):
        print('block request handler working')
        msg = block_request_t.decode(data)
        block_rq = block_request_t()
        block_rq.new_block = msg.new_block
        if self.block_counter != msg.new_block:
            if len(self.arm_block_list) == 0:
                print("neq send -1")
                block_rq.block.tag_id = -1
            else:
                print("send next")
                for haha in self.arm_block_list:
                    print("haha = ", haha.tag_id, haha.pose.x)
                self.cur_block = self.arm_block_list.pop(0)
                self.arm_block_ids.pop(0)
            self.block_counter = msg.new_block
            block_rq.block = self.cur_block
            print("cur_block_id:")
            print(self.cur_block.tag_id)
        else:
            print("send same")
            block_rq.block = self.cur_block
        print("tag_id:")
        print(block_rq.block.tag_id)
        for i in range(0,3):
            self.lc.publish(NEXT_BLOCK_RETURN_CH, block_rq.encode())
        if not self.initialized:
            self.initialized = 1

    def slam_pose_handler(self, channel, data):
        msg = pose_xyt_t.decode(data)
        self.cur_pose.x = msg.x
        self.cur_pose.y = msg.y
        self.cur_pose.theta = msg.theta

    def path_complete_handler(self,channel,data):
        print('path_complete_handler working')
        msg = path_status_t.decode(data)
        if msg.completed != self.complete_counter:
            self.complete_counter = msg.completed
            self.path_completed = True

    def calibrate_extrinsics(self, verbose = True):
        """Perform camera extrinsics calibration here
        TODO: Use appropriate cameraMatrix (intrinsics) parameters
        Change the arm frame 3d coordinate based on the point that you choose.
        Store the extrinsic matrix and load it on start.
        """
        cameraMatrix = np.array([[610, 0, 320], [0, 610, 240], [0, 0, 1]])
        tag_size = 0.02582
        # 3D coordinates of the corners of the calibration object in the world frame.
        objectPoints = np.array([[0.16, -tag_size/2, -0.060-tag_size/2],
                                 [0.16, tag_size/2, -0.060-tag_size/2],
                                 [0.16,  tag_size/2, -0.060+tag_size/2],
                                 [0.16,  -tag_size/2, -0.060+tag_size/2]])

        # Use the center of the tags as image points. Make sure they correspond to the 3D points.
        if(len(self.tags)==1):
            for tag in self.tags:
                imagePoints = tag.corners
                print("tag corners:")
                print(tag.corners)
                success, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, None)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                self.camera_extrinsics[0:3,0:3] = rotation_matrix
                self.camera_extrinsics[0:3,3] = np.transpose(tvec)
                self.camera_extrinsics[3,3] = 1
                print(np.transpose(rotation_matrix))
                print(np.transpose(tvec))

                pose = np.zeros((4,4))
                pose[0:3,0:3] = tag.pose_R
                pose[0:3,3] = np.transpose(tag.pose_t)
                pose[3,3] = 1.0
                if (verbose):
                    print("pose camera frame:")
                    print(pose)
                    print("pose world frame:")
                    print(self.camera_extrinsics.dot(np.vstack((tag.pose_t,[1]))))
                    print(self.camera_extrinsics.dot(pose))
                    print("pose world frame 2:")
                    print(pose.dot(self.camera_extrinsics))
        else:
            print("ERROR in calibrate_extrinsics: too many tags visible")

        # Set the next state to be idle
        self.set_current_state("idle")

    def robot_path_gen(self, x, y, theta, flagRalative):

        self.robot_path.path[0].x = self.cur_pose.x
        self.robot_path.path[0].y = self.cur_pose.y
        self.robot_path.path[0].theta = self.cur_pose.theta
        if flagRalative:
            self.robot_path.path[1].x = self.cur_pose.x + x * np.cos(self.cur_pose.theta) - y * np.sin(self.cur_pose.theta)
            self.robot_path.path[1].y = self.cur_pose.y + x * np.sin(self.cur_pose.theta) + y * np.cos(self.cur_pose.theta)
            self.robot_path.path[1].theta = self.cur_pose.theta + theta
        else:
            self.robot_path.path[1].x = x
            self.robot_path.path[1].y = y
            self.robot_path.path[1].theta = theta
