import lcm
import time
import select
from camera_message_lcmt import camera_message_lcmt
import numpy as np
from PIL import Image
import struct
import io

class CameraTestClass:
    def __init__(self, lc):
        self.lc = lc
        self.t = time.time()

        for i in range(5):
            self.cam_data_subscription = self.lc.subscribe("camera" + str(i), self._camera_cb)

    def _camera_cb(self, channel, data):
        msg = camera_message_lcmt.decode(data)
       
        img = np.fromstring(msg.data, dtype=np.uint8)
        img = img.reshape((3, 400, 928)).transpose(1, 2, 0)
        im = Image.fromarray(img).convert('RGB')
        im.save("test_image_" + channel + ".jpg")
        #print("ok")

    def poll(self, cb=None):
        t = time.time()
        try:
            while True:
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    #print("message received!")
                    self.lc.handle()
                    print('Frequency: ' +  str(1. / (time.time() - t)) + ' Hz'); t = time.time()
                else:
                    continue
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    ctc = CameraTestClass(lc)
    ctc.poll()


