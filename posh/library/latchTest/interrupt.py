import random

# I'm fairly certain this class is only for running the experiments Ralphshagen ran.  
# Normally agents using flexible latching should not include this or use any of its methods.
# The important methods are in POSH/latch.py
# JJB -- Feb 2008

class interrupt:
    
    def __init__(self,limit,net_increment,num_interrupts):
        self._limit=limit
        self._net_increment=net_increment
        self._num_interrupts=num_interrupts
        self._count=0
        self._currently_active=False
        self._cutoff=None
        
    def increase_count(self):
        self._count+=1
        
    def reset_count(self):
        self._count=0
        self._currently_active=False
        self._cutoff=None
        
    def should_interrupt(self,current_state):
        if not self._currently_active:
            self._currently_active=True
            
            times_called=int((self._limit-current_state)/self._net_increment)
            self._cutoff=current_state+(random.randint(0,times_called))            
        
        if self._count<self._num_interrupts and current_state>=self._cutoff:
            self._currently_active=False
            return True
        else:
            return False