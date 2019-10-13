import os
import webview

"""
iGrip GUI
"""

class Api():
    def helloGrip(self, msg):
        print('Website says: %s' % msg)


class iGripGui():
    def __init__(self):
        self.api = Api()

    def run(self):
        webview.create_window('iGrip', 'gui/assets/index.html', js_api=self.api, min_size=(600, 450))
        webview.start()
