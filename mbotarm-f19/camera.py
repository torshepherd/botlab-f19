from picamera.array import PiRGBArray
from picamera import PiCamera
from apriltags3 import Detector
import cv2
import numpy as np
import time

class MBotCamera():

    def __init__(self):
        self.camera = PiCamera()
        # Can be set to 1920 x 1080, but it will be slow.
        self.camera.resolution = (1280, 960)
        self.camera.framerate = 5
        # Wait for the automatic gain control to settle
        time.sleep(2)
        # turn off auto exposure and white balance,
        # otherwise RBG/HSV will be changing for the same object
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        awg_gain = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        # TODO: Use fixed awb_gains values instead of
        # using some random initialized value.
        self.camera.awb_gains = awg_gain

        self.rawCapture = PiRGBArray(self.camera, size=(1280, 960))
        self.detector = Detector("tagStandard41h12", quad_decimate=2.0, quad_sigma=1.0, debug=False)
        # TODO: load your camera parameters here. These camera parameters are intrinsics.
        #self.camera_params=[610, 610, 320, 240] # [fx, fy, cx, cy]
        self.camera_params=[1220, 1220, 640, 480] # [fx, fy, cx, cy]
        self.tag_size = 0.014

    def capture(self):
        self.camera.capture(self.rawCapture, format='rgb')
        image = self.rawCapture.array
        self.rawCapture.truncate(0)
        return image

    def detect_apriltags(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        tags = self.detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)
        return tags

    def display_apriltags(self, tags, image):
        apriltag_image = np.copy(image)
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(apriltag_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (255, 0, 0))

            # label the id of AprilTag on the image.
            cv2.putText(apriltag_image, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1,
                        color=(255, 0, 0))
        return apriltag_image
