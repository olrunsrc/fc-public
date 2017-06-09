from __future__ import print_function

import web
import threading
import mmap

urls = ( 
  '/', 'index',
  '/exit', 'serverexit',
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
  "message": "This is a string in double-quotes."
}
'''
        web.header('Content-Type', 'application/json')
        return ans % (id)

    def POST(self,id):
	print( "Id posted = %s"%id )
        ans = "*"
	data = web.data()
        print( web.input() )
	item = Qitem(data, ans, self.event)
	Pages.theSmartie.queue.put(item)
	self.event.wait()
        print( "Returned %s local %s" % (item.resp,ans) )
	self.event.clear()
	return "Stepped"

class serverexit:
    def __init__(self):
	self.event = threading.Event()
    def GET(self):
	item = Qitem({'msg':'exit'}, ans, self.event)
	Pages.theSmartie.queue.put(item)
	self.event.wait()
        print( "Returned %s local %s" % (item.resp,ans) )
	self.event.clear()
	return "Exited"
        
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

