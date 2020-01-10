import datetime
import time
import random
from websocket_server import WebsocketServer
import threading


class WebTunnel(threading.Thread):
    def __init__(self, controller, port):
        threading.Thread.__init__(self)

        self.port = port

        self.outboundPackets = {
            "SET_VALUE_BOUNDS": "svb",
            "FEEDBACK_VALUE": "fv" #aka true value
        }

        self.inboundPackets = {
            "USER_VALUE":"uv",
            "HEARTBEAT":"hb"
        }

        self.positionSuffix = "0";
        self.maxVelocitySuffix = "1";
        self.stiffnessSuffix = "2";

        self.jointNoToWidgetPrefix = [
            "joint0_",
            "joint1_"
        ]

        self.controller = controller
        self.queue = []

        self.server = WebsocketServer(self.port)

        self.server.set_fn_new_client(self.new_client)
        self.server.set_fn_client_left(self.client_left)
        self.server.set_fn_message_received(self.message_received)

        self.positionFeedback = {};
        self.stiffnessFeedback = {};

        self.positionBounds = {};
        self.stiffnessBounds = {};
        self.maxVelocityBounds = {};


    def run(self):
        print("Tunnel now running!")
        self.server.run_forever()
    
    def shutdown(self):
        print("Shutting down")
        try:
            self.server.shutdown()
        except:
            exit(1)

    # Called for every client connecting (after handshake)
    def new_client(self, client, server):
        print("New client connected and was given id %d" % client['id'])
        print("Filling in client with latest data")

        #send latest position bounds
        for k in self.positionBounds:
            self.server.send_message(client, self.getPositionBoundsPacket(k,
            str(self.positionBounds[k][0]), self.positionBounds[k][1]))

        #send latest position feedbacks
        for k in self.positionFeedback:
            self.server.send_message(client, self.getPositionFeedbackPacket(k,
            self.positionFeedback[k]))

        #send latest max velocity bounds
        for k in self.maxVelocityBounds:
            self.server.send_message(client, self.getMaxVelocityBoundsPacket(k,
            str(self.maxVelocityBounds[k][0]), self.maxVelocityBounds[k][1]))

        #send latest stiffness bounds
        for k in self.stiffnessBounds:
            self.server.send_message(client, self.getStiffnessBoundsPacket(k,
            str(self.stiffnessBounds[k][0]), self.stiffnessBounds[k][1]))

        #send latest applied force feedbacks
        #for k in self.stiffnessFeedback:
        #    self.server.send_message(client, self.getStiffnessFeedbackPacket(k,
        #    self.stiffnessFeedback[k]))


    def client_left(self, client, server):
	    print("Client(%d) disconnected" % client['id'])

    def message_received(self, client, server, message):
        print("Client %d said: %s", client['id'], message)

        if '%' in message:
            tokens = message.split('%')
            if tokens[0] == self.inboundPackets['USER_VALUE']:
                if tokens[1] and tokens[2]:
                    jointType = tokens[1].split("_")
                    jointType = jointType[len(jointType) - 1]

                    jointPrefix = tokens[1].split("_")[0] + "_"
                    #jointNo = jointPrefix.split("joint")
                    #jointNo = jointNo[len(jointNo) - 1]

                    print "jointPrefix: " + jointPrefix + "\n"
                    print "joint type: " + jointType + "\n"

                    if jointPrefix in self.jointNoToWidgetPrefix:
                        index = self.jointNoToWidgetPrefix.index(jointPrefix)
                           
                        if self.controller:
                            if jointType == self.positionSuffix:
                                self.controller.setPosition(index, tokens[2])
                            elif jointType == self.maxVelocitySuffix:
                                self.controller.setMaxVelocity(index, tokens[2])
                            elif jointType == self.stiffnessSuffix:
                                self.controller.setStiffness(index, tokens[2])
                        else:
                            print('Undefined controller -- tried calling setPosition(' 
                            + str(index) + ', ' + str(tokens[2]) + ')')
                    else:
                        print('Could not find target with prefix: ' + jointPrefix)
                else:
                    print('Corrupt USER_VALUE packet')
            elif tokens[0] == self.inboundPackets['HEARTBEAT']:
                print('Heartbeat from client')
                pass
            else:
                print('Unhandled packet: ' + tokens[0])
        else:
            print('Invalid packet')


    def sendPositionBoundsPacket(self, no, lower, upper):
        self.server.send_message_to_all(self.getPositionBoundsPacket(no, lower, upper))
        self.positionBounds[no] = [lower, upper]

    def sendPositionFeedbackPacket(self, no, val):
        self.server.send_message_to_all(self.getPositionFeedbackPacket(no, val))
        self.positionFeedback[no] = val
    
    def sendStiffnessBoundsPacket(self, no, lower, upper):
        print "SENDING STIFFNESS BOUNDS: " + str(no) + " " + str(lower) + " " + str(upper)
        self.server.send_message_to_all(self.getStiffnessBoundsPacket(no, lower, upper))
        self.stiffnessBounds[no] = [lower, upper]

    def sendStiffnessFeedbackPacket(self, no, val):
        self.server.send_message_to_all(self.getStiffnessFeedbackPacket(no, val))
        self.stiffnessFeedback[no] = val

    def sendMaxVelocityBoundsPacket(self, no, lower, upper):
        self.server.send_message_to_all(self.getMaxVelocityBoundsPacket(no, lower, upper))
        self.maxVelocityBounds[no] = [lower, upper]

    def getPositionBoundsPacket(self, no, lower, upper):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["SET_VALUE_BOUNDS"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.positionSuffix  + "%" + str(lower) + "%" + str(upper))
        else:
            print("Could not find widget prefix for no = " + no)
            return None

    def getStiffnessBoundsPacket(self, no, lower, upper):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["SET_VALUE_BOUNDS"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.stiffnessSuffix  + "%" + str(lower) + "%" + str(upper))
        else:
            print("Could not find widget prefix for no = " + no)
            return None

    def getPositionFeedbackPacket(self, no, val):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["FEEDBACK_VALUE"] + "%" + self.jointNoToWidgetPrefix[no] +
            str(self.positionSuffix) + "%" + str(val))
        else:
            print("Could not find widget prefix for no = " + no)
            return None
    
    def getStiffnessFeedbackPacket(self, no, val):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["FEEDBACK_VALUE"] + "%" + self.jointNoToWidgetPrefix[no] +
            str(self.stiffnessSuffix) + "%" + str(val))
        else:
            print("Could not find widget prefix for no = " + no)
            return None

    def getMaxVelocityBoundsPacket(self, no, lower, upper):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["SET_VALUE_BOUNDS"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.maxVelocitySuffix  + "%" + str(lower) + "%" + str(upper))
        else:
            print("Could not find widget prefix for no = " + no)
            return None