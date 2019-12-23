import SimpleHTTPServer
import SocketServer
import os

PORT = 8000
HTML_DIR = 'html/igrip'

Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

web_dir = os.path.join(os.path.dirname(__file__), HTML_DIR)
os.chdir(web_dir)

httpd = SocketServer.TCPServer(("", PORT), Handler)
print("serving at port ", PORT)
httpd.serve_forever()