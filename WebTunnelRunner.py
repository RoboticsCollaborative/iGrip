import asyncio
import datetime
import random
import websockets
import random

packets = {
    "FEEDBACK_VALUE": "fv" #aka true value
}

async def consumer_handler(websocket, msg):
    async for message in websocket:
        print('client sent: ' + message)


async def producer_handler(websocket, path):
    while True:
        message = await sendUpdate()
        await websocket.send(message)

async def sendUpdate():
    await asyncio.sleep(random.random() * 1)
    #send random values intermittenly amongst the different widgets
    return packets["FEEDBACK_VALUE"] + '%widget' + str(random.randint(1,3)) +'%' + str(random.randint(1, 100))

async def server(websocket, path):
    consumer_task = asyncio.ensure_future(
        consumer_handler(websocket, path))
    producer_task = asyncio.ensure_future(
        producer_handler(websocket, path))
    done, pending = await asyncio.wait(
        [consumer_task, producer_task],
        return_when=asyncio.FIRST_COMPLETED,
    )
    for task in pending:
        task.cancel()

start_server = websockets.serve(server, "127.0.0.1", 5679)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()