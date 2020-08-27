from threading import Thread
import cv2

class VideoStream(Thread):
    """
    Gather input from a video stream in its own dedicated thread
    """
    def __init__(self, src=0):
        Thread.__init__(self)
        self.stream = cv2.VideoCapture(src)
        self.stream.set(3, 5*128)
        self.stream.set(4, 5*128)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False


    # Run this thread
    def run(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()


    # Override, block caller until this thread terminates
    def join(self, timeout=None):
        self.stop()
        Thread.join(self, timeout)


    # Stop this thread
    def stop(self):
        self.stopped = True