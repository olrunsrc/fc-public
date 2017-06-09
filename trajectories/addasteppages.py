from __future__ import print_function

import web
import threading

urls = ( 
  '/', 'index',
  '/step/(.+)', 'step' )

class index:
    def GET(self):
        return "hello, world!"

class step:
    def __init__(self):
	self.event = threading.Event()

    def GET(self,id):
        ans = '''
{
  "items": [
    {
      "key": "First",
      "value": %s
    },{
      "key": "Second",
      "value": false
    },{
      "key": "Last",
      "value": "Mixed"
    }
  ],
  "obj": {
    "number": 1.2345e-6,
    "enabled": true
  },
  "message": "Strings have to be in double-quotes."
}
'''
        return ans % (id)

    def POST(self,id):

        ans = "*"
	data = web.data()
	item = Qitem(data, ans, self.event)
	Pages.theSmartie.queue.put(item)
	self.event.wait()
        print( "Returned %s local %s" % (item.resp,ans) )
	self.event.clear()
	return "Stepped"
        
class Qitem:
    def __init__(self,r,d,e):
	self.req = r
        self.resp = d
        self.evt = e

    def __repr__(self):
        return "Qitem(%s, %s, %d)" % ( self.req, self.resp, self.evt.is_set())

class Pages():

    def __init__(self,smart):
	Pages.theSmartie = smart
        print( "In Pages globals are %s" % globals().keys() )
        self.app = web.application(urls,globals())

    def run(self):
        self.app.run()

    def finish(self):
	self.app.stop()
        pass

