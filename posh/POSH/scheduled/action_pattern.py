from __future__ import nested_scopes

from complex import Complex
from types import *
from messages import Prereq
from basic import *
from generic import Generic
from messages import Content
from sense import Sense

#a simple structure for encoding sequences or parallel sets of behaviors
#which is entirely triggered by a single environmental condition.  Once
#the pattern has started, it will complete in order unless there is a 
#catestrophic failure of an element.  In this case, the remaining pattern
#dies.
class Action_Pattern(Complex):
    
    def __init__(self,
                 ap_guts = [],
                 **kw):
        # Call ancestor init with the remaining keywords
        Complex.__init__(self, **kw)
        self.ap_guts = ap_guts
        
    # When we fire the AP on the scheduler, it should put
    # all of its elements onto the scheduler by calling activate()
    # On subsequent runs, only the elements will run.
    def fire(self,
             timestamp,
             timeout_interval = DEFAULT_VIEWTIME):
        return self.activate(timestamp)
    
    def activate(self, timestamp):
        # Put a note of this on the BB
        self.send(content = Content(command = "active",
                                    tag = self.tag,
                                    action = self.action,
                                    value = self.value,
                                    timestamp = timestamp,
                                    agent = self),
                  timeout_interval = self.timeout_interval,
                  timestamp = timestamp)
        # Now for each element. If the element is a list,
        # then the elements within the second-level list needs to
        # run parallel. This means that they will all have the
        # same precondition, which is the previous element in the top-level
        # list. The next element will have all of the elementst in the
        # second-level list as proconditions.
        next_preconditions = []
        for element in self.elements:
            this_preconditions = next_preconditions # Slide the prereqs forward
            next_preconditions = [] # and clear the way for the next prereqs
            # If the element is a List, then descend into it
            if type(element) is ListType:
                for sub_element in element:
                    p_tag = [self.tag, gensym()]
                    self.schedule_element(preconditions = this_preconditions,
                                          postactions = [
                                                   Prereq(command = "result",
                                                          tag = p_tag,
                                                          agent = self.agent)],
                                          timestamp = timestamp,
                                          element = element)
                    next_preconditions.append(Prereq(command = "done",
                                                     tag = p_tag,
                                                     agent = self.agent))
            # Else process the element regularly
            else:
                p_tag = [self.tag, gensym()]
                self.schedule_element(preconditions = this_preconditions,
                                      postactions = [Prereq(command = "result",
                                                            tag = p_tag,
                                                            agent = self.agent)
                                                     ],
                                      timestamp = timestamp,
                                      element = element)
                next_preconditions = [Prereq(command = "done",
                                             tag = p_tag,
                                             agent = self.agent)]
                
    # List Generic.make_instance but returns an Action_Pattern instance
    def make_instance(self,
                      drive_name,
                      timestamp,
                      preconditions = [],
                      postactions  = []):
        preconditions = list(preconditions)
        postactions = list(postactions)
        preconditions.extend(self.preconditions)
        postactions.extend(self.postactions)
        return Action_Pattern(drive_name = drive_name,
                              preconditions = preconditions,
                              postactions = postactions,
                              elements = self.copy_elements(drive_name),
                              name = self.name,
                              timeout_interval = self.timeout_interval,
                              timeout = timestamp + self.timeout_interval,
                              tag = self.tag,
                              command = "active",
                              action = self.action,
                              value = self.value,
                              timestamp = timestamp,
                              agent = self.agent)                
    def copy_elements(self, drive_name):
        # Make a subfunction that does the actual filtering (ie. mapcar)
        # This function is recursive but should only descend one level
        def proc_elem(this_elem):
            if type(this_elem) is ListType:
                return map(proc_elem, this_elem)
            elif (type(this_elem) is InstanceType) and \
                     isinstance(this_elem, Sense):
                return Sense(sensor = this_elem.sensor,
                             sense_value = sense_value,
                             sense_predicate = sense_predicate,
                             drive_name = drive_name,
                             agent = self.agent)
            else:
                return this_elem

        # Return the 
        return map(proc_elem, self.elements)

    # The original LISP code doesn't seem to recurse into the element list.
    # Should it? It does now. This is called when the action pattern
    # is a trigger, and all the elements in the action pattern
    # are used as a condition. This is opposed to when the action
    # pattern is fire()d (ie. put on schedule)
    def trigger(self):
        # Descend recursively into elements. If any element fails -
        # the whole thing falls

	
        def proc_items(elements):
            
            for this_elem in elements:
                if type(this_elem) is ListType:
                    return proc_elem(this_elem)
                elif (type(this_elem) is InstanceType) and \
                         isinstance(this_elem, Generic):
                    # Probably a sense
                    if not this_elem.trigger():
			
                        return 0 # if one element fails, stop
                else:
                    # If it isn't s Generic object, then it must be a act
                    if not this_elem():
			
                        return 0 # if one element fails, stop
            
            return 1
        
        
        return proc_items(self.elements)