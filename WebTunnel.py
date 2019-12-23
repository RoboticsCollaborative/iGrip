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
            "USER_VALUE":"uv"
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
        self.positionBounds = {};


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

    def client_left(self, client, server):
	    print("Client(%d) disconnected" % client['id'])

    def message_received(self, client, server, message):
        print("Client %d said: %s", client['id'], message)

        if '%' in message:
            tokens = message.split('%')
            if tokens[0] == self.inboundPackets['USER_VALUE']:
                if tokens[1] and tokens[2]:
                    if tokens[1][:-1] in self.jointNoToWidgetPrefix:
                        index = self.jointNoToWidgetPrefix.index(tokens[1][:-1])
                           
                        if self.controller:
                            self.controller.setPosition(index, tokens[2])
                        else:
                            print('Undefined controller -- tried calling setPosition(' 
                            + str(index) + ', ' + str(tokens[2]) + ')')
                    else:
                        print('Could not find target with prefix: ' + tokens[1][:-1])
                else:
                    print('Corrupt USER_VALUE packet')
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

    def getPositionBoundsPacket(self, no, lower, upper):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["SET_VALUE_BOUNDS"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.positionSuffix  + "%" + str(lower) + "%" + str(upper))
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