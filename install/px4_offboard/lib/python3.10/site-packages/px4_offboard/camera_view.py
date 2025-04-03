import cv2
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
import numpy as np
from PIL import Image as PIL_Image
import threading

class Camera:
    def __init__(self, topic_name, resolution):
        self.resolution = resolution
        self.node = Node()
        self.node.subscribe(Image, topic_name, self._cb)
        self.image = None
        self.condition = threading.Condition()


    def _cb(self, image: PIL_Image) -> None:
        raw_image_data = image.data
        np_image = np.frombuffer(raw_image_data, dtype=np.uint8).reshape((self.resolution[1], self.resolution[0], 3))
        cv2_image = cv2.cvtColor(np_image, cv2.COLOR_RGB2BGR)    

        with self.condition:
            self.image = cv2_image
            self.condition.notify_all()


    def get_next_image(self, timeout=None):
        with self.condition:
            if self.image is None:
                self.condition.wait_for(lambda: self.image is not None, timeout=timeout)

            return_image = self.image
            self.image = None

            return return_image


if __name__ == "__main__":
    cam = Camera("/camera", (1920,1080))

    while True:
        img = cam.get_next_image()
        cv2.imshow('pic-display', img)
        cv2.waitKey(1)