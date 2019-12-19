import asyncio
import datetime
import random
import websockets
import random
import threading


class WebTunnel(threading.Thread):
    def __init__(self, controller):
        threading.Thread.__init__(self)

        self.outboundPackets = {
            "SET_BOUNDS": "sb",
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

        self.loop = asyncio.new_event_loop()
        #self.queue.append(self.createPositionBoundsPacket(0, 10, 100))


    async def consumer_handler(self, websocket, msg):
        async for message in websocket:
            print('client sent: ' + message)
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

    async def producer_handler(self, websocket, path):
        while True:
            message = await self.sendUpdate()

            if message == None:
                await asyncio.sleep(random.random() * 1)
                continue

            await websocket.send(message)

    def createPositionBoundsPacket(self, no, lower, upper):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["SET_BOUNDS"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.positionSuffix  + "%" + str(lower) + "%" + str(upper))
        else:
            print("Could not find widget prefix for no = " + no)
            return None

    def createPositionFeedbackPacket(self, no, val):
        if self.jointNoToWidgetPrefix[no]:
            return (self.outboundPackets["FEEDBACK_VALUE"] + "%" + self.jointNoToWidgetPrefix[no] +
            self.positionSuffix + "%" + val)
        else:
            print("Could not find widget prefix for no = " + no)
            return None

    async def sendUpdate(self):
        if len(self.queue) > 0:
            print('found queued packet to send')
            return self.queue.pop(0)
        return None
    
    #pushes string packet to queue
    def pushPkt(self, pkt):
        self.queue.append(pkt)

    async def server(self, websocket, path):
        consumer_task = asyncio.ensure_future(
            self.consumer_handler(websocket, path))
        producer_task = asyncio.ensure_future(
            self.producer_handler(websocket, path))
        done, pending = await asyncio.wait(
            [consumer_task, producer_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for task in pending:
            task.cancel()

    def run(self):
        print("Tunnel now running!")
        asyncio.set_event_loop(self.loop)

        self.start_server = websockets.serve(self.server, "127.0.0.1", 5679)

        asyncio.get_event_loop().run_until_complete(self.start_server)
        asyncio.get_event_loop().run_forever()