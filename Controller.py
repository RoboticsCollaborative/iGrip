import rospy
import numpy as np
import threading


from RddaProxy3 import RddaProxy3

class Controller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rddaProxy = RddaProxy3()
        self.isRunning = False
        #TODO: make these modular and dynamically loaded
        self.stiffness = np.array([0.5, 0.5])
        self.position = np.array([-0.5, -0.5])
        self.rate = rospy.Rate(500)
        print("Initialized controller thread")

    def run(self):
        print("Controller started!")
        self.isRunning = True
        while not rospy.is_shutdown():
            if not self.isRunning:
                break

            self.rddaProxy.set_stiffness(self.stiffness)
            self.rddaProxy.set_positions(self.position)
            self.rate.sleep()
    
    def stop(self):
        self.isRunning = False