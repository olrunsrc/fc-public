#!/usr/bin/env python
from __future__ import print_function

import time
import sys
import getopt
import numpy as np
import posix_ipc
import mmap
import json
import math

#    each comm gets a shared memory(some size), a queue and a semaphore and a memmap for read/write
#

class IPComm:

  Kib = 1024
  Mib = Kib*Kib

  def __init__(self,name,create=False,size=1024):
    self.name = name
    self.writecnt = 0

    even = (size % posix_ipc.PAGE_SIZE) == 0
    newsize = size
    if not even:
        newsize = int(math.ceil(size / posix_ipc.PAGE_SIZE)+1)*posix_ipc.PAGE_SIZE

    memory = posix_ipc.SharedMemory("/ipcomm"+name+"mmap", posix_ipc.O_CREAT, size=newsize)
    self.sem = posix_ipc.Semaphore("/ipcomm"+name+"sem", posix_ipc.O_CREAT, initial_value = 1)
    self.queue = posix_ipc.MessageQueue("/ipcomm"+name+"queue", posix_ipc.O_CREAT)
    self.mmap = mmap.mmap(memory.fd, memory.size)
    memory.close_fd()

  def empty(self):
    return( self.queue.current_messages == 0 )

  def __del__(self):
    try:
	self.mmap.close()
	self.queue.close()
	posix_ipc.unlink_shared_memory("/ipcomm"+self.name+"mmap")
	self.sem.release()
	self.sem.unlink()
	self.queue.unlink()
    except Exception as e:
	print("Error shutting down IPComm %s: %s"%(self.name,e))

def main(args):
     sample = IPComm("testOK",size=2*IPComm.Kib)
     sample.queue.send("Queued Message")
     with sample.sem as sem:
         print(sem.name)
         sample.mmap.seek(0)
         sample.mmap.write("Test data")
     if sample.queue.current_messages > 0:
       print(sample.queue.receive())
     if sample.empty:
       print("Queue is empty")
     with sample.sem as sem:
         sample.mmap.seek(0)
         print(sample.mmap.readline())
     sample = None

if __name__ == '__main__':
    main(sys.argv)
