import time
from WebTunnel import WebTunnel

tunnel = WebTunnel(None, 5679)
tunnel.start()

tunnel.sendPositionFeedbackPacket(0, 0.6432)
tunnel.sendPositionBoundsPacket(0, -1.2, 1.3)


while True:
    try:
        input = raw_input("Type in q to quit\n")
    except KeyboardInterrupt:
        tunnel.shutdown()
        break

    if input == 'q':
        tunnel.shutdown()
        break

