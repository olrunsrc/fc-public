import web

global theSmartie

urls = ( '/', 'index' )

class index:
    def GET(self):
        return "hello, world!"

class Pages():
    def __init__(self):
        self.app = web.application(urls,globals())

    def run(self):
        self.app.run()

    def finish(self):
	self.app.stop()
        pass

