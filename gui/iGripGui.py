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
        webview.create_window('iGrip', 'gui/assets/index.html', width=900, height=700,
                resizable=False, js_api=self.api)
        webview.start()
