#!/usr/bin/env python
###############################################################################
#
# The MIT License (MIT)
#
# Copyright (c) Crossbar.io Technologies GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
###############################################################################
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
from ipcomm import IPComm

global ipcWS
def getSrcString():
    global ipcWS
    try:
      res = "oops"
      with ipcWS.sem:
	ipcWS.mmap.seek(0)
	jsonstr = ipcWS.mmap.readline()
	res = "{\"SRC\": %s }" % jsonstr
      return res
    except Exception as e:
	print(e)

class Repeater:
    def __init__(self, coro, intv=0.0):
	self.done = False
	self.loop = trollius.get_event_loop()
	self.coro = coro
	self.running = trollius.Event()
	self.interval = 0.0
	self.restart(intv)
	#self.run()
	self.loop.create_task(self.run())

    @trollius.coroutine
    def run(self):
      while not self.done:
	if not self.running.is_set():
	    print("Waiting at %6.3f" % (self.loop.time()))
	    yield self.running.wait()
	    print("Run at %6.3f interval=%6.3f" % (self.loop.time(),self.interval))
	if self.interval > 0.0:
	    self.loop.create_task(self.coro())
	    #self.handler = self.loop.call_later(self.interval, self.run )
            yield trollius.sleep(self.interval)

    def restart(self,intv):
	print("Restart at %6.3f old=%6.3f new=%6.3f" % (self.loop.time(),self.interval,intv))
	self.interval = float(intv)
	self.running.set() if intv > 0.0 else self.running.clear()

    def stop(self):
	print("Stop at %6.3f" % (self.loop.time()))
	self.restart(0.0)


class ChallengeProtocolA(WebSocketServerProtocol):

    @trollius.coroutine
    def slowsquare(self, x):
        if x > 5:
            raise Exception("number too large")
        else:
            yield trollius.sleep(1)
            raise trollius.Return(x * x)

    @trollius.coroutine
    def getSquare(self, msg):
	    print(msg)
            try:
                val = float(msg.get('x',0.1))
                print("Request to square %f received. " % val)
            except Exception as e:
                print(e)
            try:
                res = yield self.slowsquare(val)
            except Exception as e:
                self.sendClose(1000, "Exception raised: {0}".format(e))
            else:
                self.sendMessage(json.dumps({'res': res}).encode('utf8'))

    @trollius.coroutine
    def getSrc2(self):
	try:
	    res = getSrcString()
            yield self.sendMessage(res.encode('utf8'))
        except Exception as e:
            print(e)

    @trollius.coroutine
    def getSrc(self,msg):
	try:
	    val = float(msg.get('x',0))
	    try:
		self.srcforever.restart(val)
		print("SRC at %6.3f with interval=%6.3f" % (trollius.get_event_loop().time(),val))
            except AttributeError as e:
		print("Create Repeater at %6.3f" % (trollius.get_event_loop().time()))
		self.srcforever = Repeater(self.getSrc2,val)
            #yield self.getSrc2()
        except Exception as e:
            print(e)

    @trollius.coroutine
    def getTask(self,msg):
 	global ipcWS
	try:
	    val = str(msg.get('x',' '))
	    res =ipcWS.queue.send("msg=%s"%val)
            print("Sent tsk %s, returned %s"%(val,res))
        except Exception as e:
            print(e)

    @trollius.coroutine
    def onMessage(self, payload, isBinary):
        if not isBinary:
            #print(payload)
            msg = json.loads(payload) #//.decode('utf8'))
            #print(msg)
	    cmd = str(msg.get('cmd','sqr'))
	    print(cmd)
	    if cmd == 'sqr':
		yield self.getSquare(msg)
	    elif cmd == 'SRC': 
		yield self.getSrc(msg)
	    elif cmd == 'tsk': 
		yield self.getTask(msg)
	else:
	    print("CPAWebSockets received binary message")

    @trollius.coroutine
    def onConnect(self, request):
        print("CPAClient connecting: {0}".format(request.peer))

    @trollius.coroutine
    def onOpen(self):
        print("CPAWebSocket connection open.")

    @trollius.coroutine
    def onClose(self, wasClean, code, reason):
	print("CPAWebSocket connection closed: {0}".format(reason))

class WSS:
    def __init__(self,protocol,ip='127.0.0.1',port=9002):
	global ipcWS
	ipcWS = IPComm("chalmap")
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
            self.loop.stop()
	    self.loop.close()
	except:
	    print("WebSockets closed")

def main():
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(threadName)10s %(name)18s: %(message)s',
        stream=sys.stderr,
    )
    ip = sys.argv[1]
    srv = WSS(ChallengeProtocolA, ip, 9002)
    try:
        srv.run()
    except KeyboardInterrupt:
        pass
    finally:
        srv.fini()

if __name__ == '__main__':
    main()

