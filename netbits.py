#!/usr/bin/env python
from __future__ import print_function

import sys
import time
import collections

class avgN():
  def __init__(self,N):
    self.N = N
    self.dq = collections.deque(N*[0.0],N)

  def next(self,val):
    self.dq.append(val)
    return sum(self.dq)/self.N

  def avg(self):
    return sum(self.dq)/self.N

  def sum(self):
    return sum(self.dq)

def update(qs,delta,elapsed):
  res = [fmtMegs(delta)]
  res.append(fmtMegs(qs[0].next(delta)))
  res.append(fmtMegs(qs[1].next(delta)))
  if elapsed % 60 == 0:
    mindelta = qs[1].sum()/60
    res.append(fmtMegs(qs[2].next(mindelta)))
    res.append(fmtMegs(qs[3].next(mindelta)))
  else:
    res.append(fmtMegs(qs[2].avg()))
    res.append(fmtMegs(qs[3].avg()))
  return res

def fmtMegs(val):
  fmt = 'B'
  if val >= 1000000:
    fmt = 'M'
    val = val/1000000
  elif val >= 1000:
    fmt = 'K'
    val = val/1000
  res = '        '+'%6.3f%s'%(val,fmt)
  return (res[-9:])

def main():
  starttm = time.time()
  rxqs = [avgN(6), avgN(60), avgN(10), avgN(60)]
  txqs = [avgN(6), avgN(60), avgN(10), avgN(60)]
  rx_bytes, tx_bytes = get_network_bytes(sys.argv[1])
  while True:
    time.sleep(1.0)
    elapsed = int(time.time()-starttm)
    newrx_bytes, newtx_bytes = get_network_bytes(sys.argv[1])
    rxcur = update(rxqs,newrx_bytes-rx_bytes,elapsed)
    txcur = update(txqs,newtx_bytes-tx_bytes,elapsed)
    rx_bytes = newrx_bytes
    tx_bytes = newtx_bytes
    if elapsed % 6 == 0:
      print( "Received 1sec:%8s 6sec:%s 1min:%s 10min:%s 1hr:%s" % tuple(rxcur) )
      print( "Transmit 1sec:%8s 6sec:%s 1min:%s 10min:%s 1hr:%s" % tuple(txcur) )


      
def get_network_bytes(interface):
    for line in open('/proc/net/dev', 'r'):
        if interface in line:
            data = line.split('%s:' % interface)[1].split()
            rx_bytes, tx_bytes = (data[0], data[8])
            return (int(rx_bytes), int(tx_bytes))

if __name__ == '__main__':
    main()
