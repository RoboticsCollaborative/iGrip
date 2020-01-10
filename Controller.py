import rospy
import numpy as np
import threading

from RddaProxy import RddaProxy
from WebTunnel import WebTunnel


class Controller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rddaProxy = RddaProxy()
        self.isRunning = False
        #TODO: make these modular and dynamically loaded
        self.stiffness = np.array([0.4, 0.4])
        self.position = np.array([0.15, 0.15])
        self.maxVelocity = np.array([5.0, 5.0])
        self.lastActualPositions = np.array([0.15, 0.15])
        self.rate = rospy.Rate(500)
        #setup web tunnel
        self.tunnel = WebTunnel(self, 5679)
        self.iterations = 0

        print("Initialized controller thread")

    def run(self):
        print("Controller started!")
        self.isRunning = True

        print("Homing..")
        self.rddaProxy.homing()

        print("Done homing. Starting web tunnel")
        self.tunnel.start()

        for i in range(len(self.rddaProxy.joint_lower_bounds)):
            #position bounds & values
            self.tunnel.sendPositionBoundsPacket(i, self.rddaProxy.joint_lower_bounds[i], self.rddaProxy.joint_upper_bounds[i])
            self.tunnel.sendPositionFeedbackPacket(i, self.rddaProxy.actual_positions[i])

            #stifness bounds
            self.tunnel.sendStiffnessBoundsPacket(i, 0, 5)
            self.tunnel.sendMaxVelocityBoundsPacket(i, 0, 10)

        while not rospy.is_shutdown():
            if not self.isRunning:
                self.tunnel.shutdown()
                break

            self.rddaProxy.set_stiffness(self.stiffness)
            self.rddaProxy.set_positions(self.position)
            self.rddaProxy.set_max_velocities(self.maxVelocity)

            if self.iterations % 2 == 0:
                for i in range(len(self.rddaProxy.joint_lower_bounds)):
                    if abs(self.lastActualPositions[i] - self.rddaProxy.actual_positions[i]) < 0.001:
                        pass
                    else:
                        self.lastActualPositions[i] = self.rddaProxy.actual_positions[i]
                        self.tunnel.sendPositionFeedbackPacket(i, self.rddaProxy.actual_positions[i])

                    self.tunnel.sendStiffnessFeedbackPacket(i, self.rddaProxy.applied_efforts[i])

            self.rate.sleep()
            self.iterations += 1

    def setPosition(self, i, val):
        print 'Setting position i = ' + str(i) + ' val = ' + str(val)
        if i < len(self.position):
            self.position[i] = val
        else:
            print 'Index is invalid!'
    
    def setStiffness(self, i, val):
        print 'Setting stiffness i = ' + str(i) + ' val = ' + str(val)
        if i < len(self.position):
            self.stiffness[i] = val
        else:
            print 'Index is invalid!'

    def stop(self):
        self.isRunning = False
        self.tunnel.shutdown()
