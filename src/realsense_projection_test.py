# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import sys
from datetime import datetime

from camera import camera_dev_realsense, camera_videocapture

if __name__ == '__main__':

    rsCamDev = camera_dev_realsense.RealsenseCapture('001622072547')
    if rsCamDev == None:
        print("can't initialize the realsense device")
        sys.exit()

    # set clipping distance
    rsCamDev.setClipping(1.5)

    # set to apply post-filters
    rsCamDev.set_flag_filters(True)
    rsCamDev.prepare_filters()

    # create video capture object using realsense camera object
    vcap = camera_videocapture.VideoCapture(
        rsCamDev, 1280, 720, 30, 'camera02')

    vcap.start()

    # Streaming loop
    try:
        while True:

            # start: measure fps
            dt0 = datetime.now()

            (color_image, depth_image) = vcap.getFrame()

            # render color & depth images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Align Example', images)

            # start: measure fps
            process_time = datetime.now() - dt0
            print("FPS: " + str(1 / process_time.total_seconds()))

            key = cv2.waitKey(1)
            # press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        vcap.stop()
