import time
import asyncio

from WebTunnel import WebTunnel


def main():
    tunnel = WebTunnel(None)
    tunnel.start()
    print("starting webtunnel")
    print("Started webtunnel")

    print("queuing position bounds packet")
    tunnel.pushPkt(tunnel.createPositionBoundsPacket(0, 10, 100))

    time.sleep(2)
    tunnel.pushPkt(tunnel.createPositionBoundsPacket(0, 5, 100))
    tunnel.pushPkt(tunnel.createPositionBoundsPacket(1, 10, 100))


main()