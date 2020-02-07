#!/usr/bin/env python3
import lcm
import sys
import cv2
import numpy as np
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)
import qimage2ndarray

import os
import io
os.sys.path.append('dynamixel/') # Path setting
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

from ui import Ui_MainWindow
from mbot_arm import MBotArm
from camera import MBotCamera
from trajectory_planner import TrajectoryPlanner
from state_machine import StateMachine


""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592
#pixel Positions of image in GUI
MIN_X = 240
MAX_X = 880
MIN_Y = 40
MAX_Y = 520

""" Serial Port Parameters"""
BAUDRATE   = 1000000
DEVICENAME = "/dev/ttyACM0".encode('utf-8')

"""Threads"""
class VideoThread(QThread):
    # TODO: You can make updateFrame take more images in the list as input.
    # That way you will be emit for images to visualize for debugging.
    updateFrame = pyqtSignal(list)
    updateAprilTags = pyqtSignal(list)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera

    def run(self):
        while True:
            image = self.camera.capture()
            tags = self.camera.detect_apriltags(image)
            apriltag_image = self.camera.display_apriltags(tags, image)
            self.updateFrame.emit([image, apriltag_image])
            self.updateAprilTags.emit(tags)
            time.sleep(0.05)

class LogicThread(QThread):
    # Run the lowerest level state machine.
    def __init__(self, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        while True:
            self.sm.run()
            time.sleep(0.05)

class DisplayThread(QThread):
    updateStatusMessage = pyqtSignal(str)
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, mbot_arm, state_machine, parent=None):
        QThread.__init__(self, parent=parent)
        self.mbot_arm = mbot_arm
        self.sm=state_machine

    def run(self):
        while True:
            self.updateStatusMessage.emit(self.sm.status_message)
            self.updateJointReadout.emit(self.mbot_arm.joint_angles_fb)
            self.updateEndEffectorReadout.emit(list(self.mbot_arm.get_fb_pose()))
            time.sleep(0.1)

"""GUI Class"""
class Gui(QMainWindow):
    """
    Main GUI Class
    contains the main function and interfaces between
    the GUI and functions
    """
    def __init__(self,parent=None):
        QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Set GUI to track mouse """
        QWidget.setMouseTracking(self,True)

        """
        Dynamixel bus
        TODO: add other motors here as needed with their correct address"""
        self.dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
        port_num = self.dxlbus.port()
        base = DXL_XL(port_num, 1)
        shld = DXL_XL(port_num, 2)
        elbw = DXL_XL(port_num, 3)
        wrst = DXL_XL(port_num, 4)
        grip = DXL_XL(port_num, 5)

        """Objects Using Other Classes"""
        self.camera = MBotCamera()
        self.mbot_arm = MBotArm((base,shld,elbw,wrst,grip))
        self.tp = TrajectoryPlanner(self.mbot_arm)
        self.sm = StateMachine(self.mbot_arm, self.tp)
        self.rgb_image = None

        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btnUser1.setText("Detect Tags")
        self.ui.btnUser1.clicked.connect(partial(self.sm.set_current_state, "detect_tags"))
        self.ui.btnUser2.setText("Stop Detection")
        self.ui.btnUser2.clicked.connect(partial(self.sm.set_current_state, "stop_detection"))
        self.ui.btnUser3.setText("Calibrate Extrinsics")
        self.ui.btnUser3.clicked.connect(partial(self.sm.set_current_state, "calibrate_extrinsics"))
        self.ui.btnUser3.setText("Test Release")
        self.ui.btnUser3.clicked.connect(partial(self.sm.set_current_state, "release_block"))
        self.ui.btn_exec.setText("Test Grasp")
        self.ui.btn_exec.clicked.connect(partial(self.sm.set_current_state, "grab_block"))

        self.ui.sldrBase.valueChanged.connect(self.sliderChange)
        self.ui.sldrShoulder.valueChanged.connect(self.sliderChange)
        self.ui.sldrElbow.valueChanged.connect(self.sliderChange)
        self.ui.sldrWrist.valueChanged.connect(self.sliderChange)
        self.ui.sldrGrip1.valueChanged.connect(self.sliderChange)

        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        self.ui.rdoutStatus.setText("Waiting for input")

        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """initalize mbot_arm"""
        self.mbot_arm.initialize()

        """Setup Threads"""
        self.videoThread = VideoThread(self.camera)
        self.videoThread.updateFrame.connect(self.setImage)
        self.videoThread.updateAprilTags.connect(self.updateAprilTags)
        self.videoThread.start()


        self.logicThread = LogicThread(self.sm)
        self.logicThread.start()

        self.displayThread = DisplayThread(self.mbot_arm, self.sm)
        self.displayThread.updateJointReadout.connect(self.updateJointReadout)
        self.displayThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
        self.displayThread.updateStatusMessage.connect(self.updateStatusMessage)
        self.displayThread.start()

        """
        Setup Timer
        this runs the trackMouse function every 50ms
        """
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.trackMouse)
        self._timer.start(50)

    """ Slots attach callback functions to signals emitted from threads"""

    # Convert image to Qt image for display.
    def convertImage(self, image):
        qimg=QImage(image, image.shape[1], image.shape[0], QImage.Format_RGB888)
        return QPixmap.fromImage(qimg)

    # TODO: Add more QImage in the list for visualization and debugging
    @pyqtSlot(list)
    def setImage(self, image_list):
        rgb_image = image_list[0]
        apriltag_image = image_list[1]
        if(self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(self.convertImage(rgb_image))
            self.rgb_image = rgb_image
        if(self.ui.radioApril.isChecked()):
            self.ui.videoDisplay.setPixmap(self.convertImage(apriltag_image))
            self.rgb_image = apriltag_image


    @pyqtSlot(list)
    def updateAprilTags(self, tags):
        self.sm.tags = tags
        self.sm.new_image = True

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        self.ui.rdoutBaseJC.setText(str("%+.2f" % (joints[0]*R2D)))
        self.ui.rdoutShoulderJC.setText(str("%+.2f" % (joints[1]*R2D)))
        self.ui.rdoutElbowJC.setText(str("%+.2f" % (joints[2]*R2D)))
        self.ui.rdoutWristJC.setText(str("%+.2f" % (joints[3]*R2D)))

    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f" % (pos[2])))
        self.ui.rdoutT.setText(str("%+.2f" % (pos[3])))

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)


    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.mbot_arm.estop = True
        self.sm.set_current_state("estop")

    def sliderChange(self):
        """
        Function to change the slider labels when sliders are moved
        and to command the arm to the given position
        """
        self.ui.rdoutBase.setText(str(self.ui.sldrBase.value()))
        self.ui.rdoutShoulder.setText(str(self.ui.sldrShoulder.value()))
        self.ui.rdoutElbow.setText(str(self.ui.sldrElbow.value()))
        self.ui.rdoutWrist.setText(str(self.ui.sldrWrist.value()))
        self.ui.rdoutGrip1.setText(str(self.ui.sldrGrip1.value()))

        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")
        self.mbot_arm.set_torque_limits([self.ui.sldrMaxTorque.value()/100.0]*self.mbot_arm.num_joints, update_now = False)
        self.mbot_arm.set_speeds_normalized_global(self.ui.sldrSpeed.value()/100.0, update_now = False)
        joint_positions = np.array([self.ui.sldrBase.value()*D2R,
                           self.ui.sldrShoulder.value()*D2R,
                           self.ui.sldrElbow.value()*D2R,
                           self.ui.sldrWrist.value()*D2R,
                           self.ui.sldrGrip1.value()*D2R])
        self.mbot_arm.set_positions(joint_positions, update_now = False)

    def directControlChk(self, state):
        if state == Qt.Checked:
            self.sm.set_current_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            self.sm.set_current_state("idle")
            self.ui.SliderFrame.setEnabled(False)

    def trackMouse(self):
        """
        Mouse position presentation in GUI
        TODO: Display the rgb/hsv value
        """

        x = QWidget.mapFromGlobal(self,QCursor.pos()).x()
        y = QWidget.mapFromGlobal(self,QCursor.pos()).y()
        if ((x < MIN_X) or (x >= MAX_X) or (y < MIN_Y) or (y >= MAX_Y)):
            self.ui.rdoutMousePixels.setText("(-,-,-)")
            self.ui.rdoutRGB.setText("(-,-,-)")
        else:
            x = x - MIN_X
            y = y - MIN_Y
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,-)" % (x,y))

            #rgb = self.rgb_image[y, x] # [r, g, b] of this pixel
            #hsv_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2HSV)
            #self.ui.rdoutRGB.setText("(R,G,B)")

    def mousePressEvent(self, QMouseEvent):
        """
        Function used to record mouse click positions for calibration
        """

        """ Get mouse posiiton """
        x = QMouseEvent.x()
        y = QMouseEvent.y()

        """ If mouse position is not over the camera image ignore """
        if ((x < MIN_X) or (x > MAX_X) or (y < MIN_Y) or (y > MAX_Y)): return

"""main function"""
def main():
    app = QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
