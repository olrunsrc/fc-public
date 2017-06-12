from __future__ import nested_scopes

from complex import *
from messages import Prereq

# provides for "reactive plans" --- prioritized sets of procedures
# or subgoals for attaining a particular goal, each with preconditions 
# which determine their applicability.
class Competence(Complex):
    def __init__(self,
                 goal = None,
                 temp_preconditions = [],
                 **kw):
        # Call ancestor init with the remaining keywords
        Complex.__init__(self, **kw)
        self.goal = goal
        self.temp_preconditions = temp_preconditions
    # Just sends the message to the BB and makes a copy of the
    # preconditions
    def activate(self, timestamp):
        self.send(content = Content(command = "active",
                                    tag = self.tag,
                                    timestamp = timestamp,
                                    action = self.action,
                                    value =  self.value,
                                    agent = self.agent),
                  timeout_interval = self.timeout_interval,
                  timestamp = timestamp)
        self.active = 1
        self.temp_preconditions = self.preconditions
    # Issues a BB message on the status change and deflag
    # self.active
    def deactivate(self, command, timestamp):
        self.send(content = Content(command = command,
                                    tag = self.tag,
                                    timestamp = timestamp,
                                    action = self.action,
                                    value =  self.value,
                                    agent = self.agent),
                  timeout_interval = self.timeout_interval,
                  timestamp = timestamp)
        self.active = 0
	 
#; because scheduler will have already fired this, we have to add it
#; back on to the schedule until it's done... so add both your chosen
#; element and yourself, with post and preconditions set up so the
#; element runs first.  "temp-preconditions" is the regular
#; preconditions for the competence (see "activate" above)
#; Notice this requires that each element 1) has it's name in its
#; core element and 2) stands as an identity with that same name in 
#; the global environment.  This is messy, but the best thing I can
#; think of to match between Edmund's need to keep track of the number
#; of times an individual element has been triggered, and Ymir's use of
#; the environment.
    def fire(self, timestamp, timeout_interval = None):
        if not self.active:
            self.activate(timestamp)
        if self.goal.ready():
            self.deactivate(command = "done", timestamp = timestamp)
        else:
            for this_elem in self.elements:
                if type(this_elem) is ListType:
                    # Descend into first level
                    for this_sub_elem in this_elem:
                        if this_sub_elem.ready() and \
                               this_sub_elem.retries != 0:
                            return self.fire_cel(
                                competence_element = this_sub_elem,
                                timestamp = timestamp,
                                timeout_interval =  timeout_interval)
                else:
                    if this_elem.ready() and this_elem.retries != 0:
                        return self.fire_cel(competence_element =  this_elem,
                                             timestamp = timestamp,
                                             timeout_interval =  timeout_interval)
            # If we didn't find any element to fire
            return self.deactivate(command = "fail", timestamp = timestamp)
        
#; this puts the element on the schedule, and returns 'preserve to keep
#; the parent on the schedule too.  It also sets a key for the child to
#; fire as a postreq, so that at least this copy of the parent won't be
#; reactivated 'til the child is done.
    def fire_cel(self,
                 competence_element,
                 timestamp,
                 timeout_interval = None):
        t_sym = [self.tag, gensym()]
        competence_element.retries = competence_element.retries - 1
        self.schedule_element(element = competence_element.action.action,
                              preconditions = [],
                              postactions = [Prereq(command = "done",
                                                    tag = t_sym,
                                                    agent = self.agent)],
                              timestamp = timestamp)
        #self.preconditions = list(self.temp_preconditions) # Make a copy
        self.preconditions = []
        self.preconditions.insert(0, Prereq(command = "done",
                                            tag = t_sym,
                                            agent = self.agent))
        return "preserve" # Return preserve to keep parent on the schedule
    
    # List Generic.make_instance but returns an Competence instance
    def make_instance(self,
                      drive_name,
                      timestamp,
                      preconditions = [],
                      postactions  = []):
        preconditions = list(preconditions)
        postactions = list(postactions)
        preconditions.extend(self.preconditions)
        postactions.extend(self.postactions)
        return Competence(drive_name = drive_name,
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
                          timestamp = self.timestamp,
                          goal = self.goal,
                          temp_preconditions = self.temp_preconditions,
                          agent = self.agent)
    
    # Different to the AP copy in that the elements in a Competence are
    # always of class Competence_Element and we have to replicate these
    # objects.
    def copy_elements(self, drive_name):
        def proc_element(this_element):
            if type(this_element) is ListType:
                return map(proc_element, this_element)
            elif type(this_element) is InstanceType and \
                     isinstance(this_element, Competence_Element):
                return Competence_Element(trigger = this_element.trigger,
                                          drive_name = drive_name,
                                          action = this_element.action,
                                          retries = this_element.retries,
                                          agent = self.agent)
            else:
                return this_element
        return map(proc_element, self.elements)
	
