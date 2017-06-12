import socket, threading, random, time, sys
try:
	import cPickle as pickle
except:
	import pickle

class TCPRobot():
	"""
	A simple TCP-based socket robot for talking to PyrobotSimulator.
	"""
	BUFSIZE = 4096 # 2048 # 1024
	def __init__(self, host, port, startDevices=1):
		
		self.lock = threading.Lock()
		# Set the socket parameters
		self.host = host
		self.port = port
		self.addr = (host, port)
		self.type = "Pyrobot"
		# Create socket
		self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		try:
			self.socket.settimeout(1)
		except:
			print "WARN: entering deadlock zone; upgrade to Python 2.3 to avoid"
		done = 0
		while not done:
			try:
				self.socket.connect( self.addr )
				done = 1
			except:
				print "Waiting on PyrobotSimulator... at %s" % self.port
				time.sleep(1)
		#self.connectionNum = self.getItem("connectionNum:%d" % self.port)
		#self.init(startDevices)

	def send(self, message, other = None):
		if self.lock.locked():
			return None
		self.lock.acquire()
		exp = None
		if self.socket == 0: return "not connected"
		if message == "quit" or message == "exit" or message == "end" or message == "disconnect":
			self.socket.sendto(message, self.addr)
			self.socket.close()
			self.socket = 0
			self.lock.release()
			return "ok"
		else:
			self.socket.sendto(message, self.addr)
			try:
				retval, addr = self.socket.recvfrom(self.BUFSIZE)
			except:
				retval = ""
			retval = retval.strip()
			try:
				exp = pickle.loads( retval )
			except:
				exp = retval
		self.lock.release()
		return exp

	def disconnect(self):
		self.getItem("disconnect")

if __name__ == "__main__":
   port = int(sys.argv[1])
   print port
   rbt = TCPRobot('localhost',port)

   while True:
      cmd = raw_input()
      print rbt.send(cmd)

