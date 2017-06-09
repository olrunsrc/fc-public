#!/usr/bin/env python
from __future__ import print_function

from autobahn.asyncio.websocket import WebSocketServerProtocol,WebSocketServerFactory
import json
import trollius
import time
import logging
import sys
import mmap
import socket
import posix_ipc
import struct
import getopt

global flyermmap, flyersem, flyerqueue

class FlyerWSP(WebSocketServerProtocol):

    @trollius.coroutine
    def onMessage(self, payload, isBinary):
	global flyerqueue
        if not isBinary:
            try:
              msg = json.loads(payload)
              #print(msg)
            except Exception as e:
              print(e)
	    #cmd = str(msg.get('cmd','X'))
	    #r = float(msg.get('r',0.0))
	    #t = float(msg.get('t',0.0))
	    #d = (cmd,r,t)
	    #print("Got (%s,%6.3f,%6.3f)" % d)
            needlock = True
            while(needlock):
              try:
	        flyerqueue.send(payload,0)
	        needlock = False
              except:
                yield trollius.sleep(0.01)
	else:
	    print("FlyerWSP: msg is binary")

    '''
    @trollius.coroutine
    def sendImages(self):
      global flyermmap, flyersem
      print("Sleeping for 5")
      yield trollius.sleep(5)
      while(self.sending):
	self.sendcnt += 1
	res = "This is message %d" % self.sendcnt
        yield self.sendMessage(res.encode('utf8'))
	print("Sleeping for 5")
   	yield trollius.sleep(5)
      print("Done sending Images") '''

    @trollius.coroutine
    def sendImages(self):
      global flyermmap, flyersem
      self.currn = -1
      while(self.sending):
        needlock = True
        while(needlock and self.sending):
          try:
	    flyersem.acquire(0)
	    #print("Got the lock")
	    needlock = False
          except:
            yield trollius.sleep(0.05)
        #have lock now
        try:
	  flyermmap.seek(0)
          buf = flyermmap.read(8)
	  l,n = struct.unpack_from('ii',buf)
	  #print("Read (%d,%d) currn is %d" % (l,n,self.currn) )
	  if self.currn != n:
	    self.currn = n 
	    self.sendcnt += 1
	    #res = "This is message %d, val=%s" % (self.sendcnt,val)
            #yield self.sendMessage(res.encode('utf8'))
	    flyermmap.seek(0)
	    res = flyermmap.read(l)
            yield self.sendMessage(res,True) #send binary message 1.6MB
	  flyersem.release()
	except Exception as e:
	  flyersem.release()
	  print("Exception %s"%e )
	#print("Sleeping for 1")
   	yield trollius.sleep(0.3)
      print("Done sending Images")

    @trollius.coroutine
    def onConnect(self, request):
        print("CPAClient connecting: {0}".format(request.peer))
	self.sendcnt = 0
	self.sending = True
	self.loop = trollius.get_event_loop()
	self.loop.create_task(self.sendImages())
	print("sendImages task created.")
	'''
	needlock = True
        while(needlock):
            try:
	        flyerqueue.send("{cmd: 'X'}",0)
	        needlock = False
            except:
                yield trollius.sleep(0.01)
	'''

    def onOpen(self):
        print("WebSocket connection open.")

    @trollius.coroutine
    def onClose(self, wasClean, code, reason):
	print("CPAWebSocket closing connection: {0}".format(reason))
	self.sending=False

class WSS:
    def __init__(self,protocol,ip='127.0.0.1',port=9000):
	global flyermmap, flyersem, flyerqueue
	self.queue = posix_ipc.MessageQueue("/flyerqueue")
	memory = posix_ipc.SharedMemory("flyermmap")
	self.sem = posix_ipc.Semaphore("flyersem")
	self.memmap = mmap.mmap(memory.fd, memory.size)
	flyermmap = self.memmap
	flyersem = self.sem
	flyerqueue = self.queue
	memory.close_fd()
        ip_str = str(ip)
        port_int = int(port)
        wsip = u"ws://" + ip_str + u":" + str(port_int)
	factory = WebSocketServerFactory(wsip)
	factory.protocol = protocol

	self.loop = trollius.get_event_loop()  
	self.loop.set_debug(False)
	self.coro = self.loop.create_server(factory, ip_str, port_int)
	self.server = self.loop.run_until_complete(self.coro)
        print("WebSockets configured on %s" % wsip)

    def run(self):
        print("WebSockets starting")
	self.loop.run_forever()
        print("WebSockets stopping")

    def fini(self):
	try:
	    self.memmap.close()
	    self.server.close()
	    self.queue.close()
            self.loop.stop()
	    self.loop.close()
	    print("WebSockets closed")
	    posix_ipc.unlink_shared_memory("flyermmap")
	    self.sem.release()
	    self.sem.unlink()
	    self.queue.unlink()
	except:
	    print("Error closing WebSockets")

def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(threadName)10s %(name)18s: %(message)s',
        stream=sys.stderr,
    )
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    port = 9000
    try:
        opts, args = getopt.getopt(sys.argv[1:], "", ["ip=","port="])
    except getopt.GetoptError as err:
        print(str(err))  # will print something like "option -a not recognized"
        sys.exit(2)
    for o, a in opts:
        if o == "--ip":
            ip = a
        if o == "--port":
            port = a
    srv = WSS(FlyerWSP, ip, port)
    try:
        srv.run()
    except KeyboardInterrupt:
        pass
    finally:
        srv.fini()

if __name__ == '__main__':
    main()
