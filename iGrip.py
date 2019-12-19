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
        self.controller = Controller()

    def run(self):
        print('iGrip is now running!')
        self.controller.start()

if __name__ == '__main__':
    try:
        rospy.init_node('iGrip')
        grip = iGrip()
        grip.run()
    except rospy.ROSInterruptException:
        pass