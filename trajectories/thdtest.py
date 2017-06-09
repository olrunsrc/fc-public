#!/usr/bin/env python
from __future__ import print_function

import time
import threading
import thread
import Queue

class Qitem:
    def __init__(self,r,d,e):
	self.req = r
        self.resp = d
        self.evt = e

    def __repr__(self):
        return "Qitem(%s, %s, %d)" % ( self.req, self.resp, self.evt.is_set())

class Listener:
    def __init__(self,thing,cnt):
        self.thing = thing
	self.event = threading.Event()
	self.cnt = cnt

    def loop(self):
    	self.POST(self.cnt)

    def POST(self,id):
        self.event.clear()
        ans = "*"
	data = id

	item = Qitem(data, ans, self.event)
	print( "POSTing %s" % item )	
	self.thing.queue.put(item)
	self.event.wait()
        print( "POST received %s local %s" % (item.resp,ans) )
	return "Stepped"

class Ros():
    def __init__(self):
        self.queue = Queue.Queue()
        self.data = 0

    def loop(self):
      while True:
        if not self.queue.empty():
            item = self.queue.get()
            self.data += 1
            print( "Queued data is %s" % item.req )
            item.resp = self.data
            item.evt.set()
            #item.evt.clear()
            self.queue.task_done()
        else:
            print("Ros: queue is empty")
	    time.sleep(1)


def mainish():
    cnt = 10

    ros = Ros()
    #web = Listener(ros)

    try:
	    thread.start_new_thread(ros.loop, ())
            while cnt > 0:
		web = Listener(ros,cnt)
		thread.start_new_thread(web.loop, ())
		#web.POST(cnt)
                time.sleep(3)
		web = None
		cnt -= 1
    finally:
	pass

if __name__ == '__main__':
  mainish()

