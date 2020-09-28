from abc import *
import enum

# video frame type enumeration


class VideoFrameType(enum.Enum):
    ColorFrame = 0
    DepthFrame = 1

# CameraPortLayer abstraction class


class CameraDev(metaclass=ABCMeta):

    ###################################
    # Properties
    ###################################
    # frame width
    _frameWidth = 0

    # frame height
    _frameHeight = 0

    # frame per sec
    _framePerSec = 0

    ###################################
    # Abstraction Functions
    ###################################
    # initialize parameters for any camera operation
    @abstractmethod
    def initialize(self, width, height, fps):
        pass

    # start to capture frames
    @abstractmethod
    def startCapture(self):
        pass

    # stop to capture frames
    @abstractmethod
    def stopCapture(self):
        pass

    # wait for a video color frame and return the frame
    @abstractmethod
    def getFrame(self):
        pass

    # get 3D position w.r.t an image pixel based on camera-based coordination
    @abstractmethod
    def get3DPosition(self):
        pass
