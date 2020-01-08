# iGrip Controller

import sys
import rospy
import threading
import numpy as np

from time import sleep
from Controller import Controller


class iGrip:
    def __init__(self):
        self.controller = Controller()

    def run(self):
        print('iGrip is now running!')
        self.controller.start()

        try:
            input = raw_input("\n\n[ Type in q to quit ]\n")
        except KeyboardInterrupt:
            self.controller.stop()

        if input == 'q':
            self.controller.stop()

if __name__ == '__main__':
    try:
        rospy.init_node('iGrip')
        grip = iGrip()
        grip.run()
    except rospy.ROSInterruptException:
        pass