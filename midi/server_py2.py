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

from autobahn.asyncio.websocket import WebSocketServerProtocol, \
    WebSocketServerFactory

import json
import trollius


class ChallengeProtocolB(WebSocketServerProtocol):

    @trollius.coroutine
    def slowsquare(self, x):
        if x > 5:
            raise Exception("number too large")
        else:
            yield trollius.sleep(1)
            raise trollius.Return(x * x)

    @trollius.coroutine
    def onMessage(self, payload, isBinary):
        if not isBinary:
            print(payload)
            msg = json.loads(payload) #//.decode('utf8'))
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

class WSS:
    def __init__(self,ip,port):
        ip_str = str(ip)
        port_int = int(port)
        wsip = u"ws://" + ip_str + u":" + str(port_int)
	factory = WebSocketServerFactory(wsip)
	factory.protocol = ChallengeProtocolB

	self.loop = trollius.get_event_loop()  
	self.coro = self.loop.create_server(factory, ip_str, port_int)
	#self.server = self.loop.run_until_complete(self.coro)
        print("WebSockets configured on %s" % wsip)

    def run(self):
	self.server = self.loop.run_until_complete(self.coro)
        print("WebSockets starting")
	self.loop.run_forever()
        print("WebSockets stopping")

    def fini(self):
	self.server.close()
	self.loop.close()


if __name__ == '__main__':
    srv = WSS('127.0.0.1', 9000)
    try:
        srv.run()
    except KeyboardInterrupt:
        pass
    finally:
        srv.fini()
