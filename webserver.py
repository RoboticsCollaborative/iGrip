import os
import http.server
import socketserver

PORT = 8080
HTML_DIR = 'html/igrip'

Handler = http.server.SimpleHTTPRequestHandler

web_dir = os.path.join(os.path.dirname(__file__), HTML_DIR)
os.chdir(web_dir)

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port ", PORT)
    httpd.serve_forever()