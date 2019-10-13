# iGrip Controller

import sys
import rospy
import threading
import numpy as np

from time import sleep
from gui.iGripGui import iGripGui
from Controller import Controller

class iGrip:
    def __init__(self):
        self.gui = iGripGui()
        self.controller = Controller()

    def run(self):
        self.controller.start()
        self.gui.run()

if __name__ == '__main__':
    try:
        rospy.init_node('iGrip')
        grip = iGrip()
        grip.run()
    except rospy.ROSInterruptException:
        pass