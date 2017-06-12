""" A class that implements a universal latch """
class Latch:
    
    """ A latch has numerous properties:
        lower=lower threshold. If current_state<lower, trigger=True
        upper=upper threshold (optional)
        inter=intermediate threshold (optional) for interrupts
        increment=positive change of current_state
        decrement=negative change of current_state        
    """
    def __init__(self,current_state,lower,increment,decrement,upper=None,inter=None,may_interrupt=False):
        self._lower=lower
        self._inter=inter
        self._upper=upper
        
        if self._upper==None:
            self._upper=self._lower
        
        self._current_state=current_state
        self._increment=increment
        self._decrement=decrement     

        self._currently_executed=False        
        self._may_interrupt=may_interrupt
    
    def wants_to_interrupt(self):
        if self._current_state<20:
            return True
        else:
            return False
    
    def decrement_current_state(self):
        self._current_state-=self._decrement
        
    def increment_current_state(self):
        self._current_state+=self._increment
    
    def get_current_state(self):
        return self._current_state
    
    def set_current_state(self,new_current_state):
        self._current_state=new_current_state
    
    def is_saturated(self):
        if self._current_state>=self._upper:
            self._currently_executed=False
            return True
        
        return False
    
    def is_triggered(self):
        return self._currently_executed or self._current_state<self._lower
    
    def signal_interrupt(self):
        if self._inter and self._current_state>self._inter:
            self._currently_executed=False
            return True
        return False
            
    def failed(self):
        return self._current_state<=0
            
    def activate(self):
        self._currently_executed=True
        
    def deactivate(self):
        self._currently_executed=False        
        
    def is_active(self):
        return self._currently_executed    
            
    def reset_agent(self):    
        self._currently_executed=False
        if self._upper:
            self._current_state=self._upper
        else:
            self._current_state=2*self._lower
        
    def get_urgency(self):
        return 1-float(self._current_state/self._lower)