from __future__ import nested_scopes

from basic import *
from message import Message
from types import *
from competence_element import Competence_Element

from POSH.utils import current_time

class Bbitem(Message):
    def __init__(self,
                 drive_name = None,
                 timeout = None,
                 **kw):
        # Call ancestor init with the remaining keywords
        Message.__init__(self, **kw)
        self.drive_name = drive_name
        self.timeout = timeout
    def make_content(self):

        return Content(command = self.command,
                       tag = self.tag,
                       action = self.action,
                       value = self.value,
                       timestamp = self.timestamp,
                       agent = self.agent)
    def make_generic(self):
        return Generic(name = self.tag,
                       content = self.make_content(),
                       drive_name = self.drive_name,
                       timeout = self.timeout,
                       agent = self.agent)
    def schedule_item(self, timestamp):
        if (type(self.action) is InstanceType) \
               and isinstance(self.action, Generic):
            self.agent.schedule.add_item(
                self.action.make_instance(drive_name = self.drive_name,
                                          timestamp = timestamp))
        elif (type(self.action) is MethodType) \
                 or (type(self.action) is FunctionType):
            self.agent.schedule.add_item(self.make_generic())
        else:
            pass # Error: Unknown Object Type



class Content(Message):
    def __init__(self,
                 timestamp = -1,
                 **kw):
        # Call ancestor init with the remaining keywords
        Message.__init__(self, **kw)
        self.timestamp = timestamp
    def make_bbitem(self,
                    drive_name,
                    timeout,
                    timestamp = None):

        if timestamp == None:
            timestamp = self.timestamp
        return Bbitem(command = self.command,
                      tag = self.tag,
                      action = self.action,
                      value = self.value,
                      timestamp = timestamp,
                      timeout = timeout,
                      drive_name = drive_name,
                      agent = self.agent)



class Generic(Base, Content):
    
    def __init__(self,
                 name = "name",
                 drive_name = "raw_prototype_no_drive",
                 preconditions = [],
                 postactions = [],
                 timeout = -1,
                 timeout_rep = None,
                 timeout_interval = DEBUG_SCHEDULE_TIME,
                 **kw):
        Base.__init__(self, **kw) # Call the base ancestor init
        Content.__init__(self, **kw) # Call the content init
        self.name = name
        self.drive_name = drive_name
        self.timeout = timeout
        self.timeout_rep = timeout_rep
        self.timeout_interval = timeout_interval
        self.preconditions = preconditions
        self.postactions = postactions
        
    # Takes in a content object and merges in the changes
    def replace_content(self, new_content):
        self.command = new_content.command
        self.tag = new_content.tag
        self.action = new_content.action
        self.value = new_content.value
        self.timestamp = new_content.timestamp
        
    # Makes a copy of the content potions of itself
    def make_content(self):
        return Content(command = self.command,
                       tag = self.tag,
                       action = self.action,
                       value = self.value,
                       timestamp = self.timestamp,
                       agent = self.agent)
    
    # Not related to LISP make-instanace, this create a clone
    # instance of itself as a Generic
    def make_instance(self,
                      drive_name = None,
                      timestamp = None,
                      preconditions = [],
                      postactions = []):
        if drive_name == None: drive_name = self.drive_name
        if timestamp == None: timestamp = self.timestamp
        # Make a copy because lists are mutable
        preconditions = list(preconditions)
        postactions = list(postactions)
        preconditions.extend(self.preconditions)
        postactions.extend(self.postactions)
        return Generic(drive_name = drive_name,
                       preconditions = preconditions,
                       postactions = postactions,
                       name = self.name,
                       timeout_interval = self.timeout_interval,
                       timeout = timestamp + self.timeout_interval,
                       tag = self.tag,
                       command = "instance",
                       action = self.action,
                       value = self.value,
                       timestamp = timestamp,
                       agent = self.agent)
    
    # Returns a bbitem from information in this object
    def make_bbitem(self,
                    command,
                    timeout,
                    timestamp = None):
        if timestamp == None:
            timestamp = current_time()
        return bbitem(command = command,
                      tag = self.tag,
                      action = self.action,
                      timestamp = timestamp,
                      drive_name = self.drive_name,
                      timeout = timeout,
                      agent = self.agent)
    
    # Puts an item on the blackboard and accepts a custom
    # content object -- ||TODO|| gernalize make_bbitem and make
    # this tie in with it (Make content optional and use info from class)
    def send(self,
             timestamp,
             content,
             timeout_interval = DEFAULT_VIEWTIME):
        self.agent.blackboard.add_item(
            content.make_bbitem(drive_name = self.drive_name,
                                timeout = current_time() +
                                    DEFAULT_VIEWTIME,
                                timestamp = timestamp))
        
    # Returns a list of item with a particular tag
    # ||TODO|| needs major restructuring
    def rec(self,
            content,
            timestamp,
            timeout_interval = -1):
        return self.agent.blackboard.find_items(
            content.make_bbitem(drive_name = self.drive_name,
                                timeout = timeoutinterval,
                                timestamp = timestamp))
    
    # Check each precondition to see if they are satisfied
    def ready(self):    
        for precon in self.preconditions:
            if not self.agent.blackboard.find_prereq(command = precon.command,
                                                     tag = precon.tag):
                return 0
        return 1

    # Fire the sucker. The function calling fire should
    # post the result to the noticeboard
    def fire(self, timestamp, timeout_interval = DEFAULT_VIEWTIME):
        try:
            result = self.action()
        except:
            print "Error in fire.  Object >> "
            print self
            print " << mamed >> %s << couldn't fire >> " % self.name
            print self.action()
            print " <<"
            raise
        if result:
            self.fire_postactions(timestamp = timestamp,
                                  timeout_interval = timeout_interval,
                                  result = "done")
        else:
            self.fire_postactions(timestamp = timestamp,
                                   timeout_interval = timeout_interval,
                                   result = "fail")
        return result
            
    def fire_postactions(self,
                         timestamp,
                         timeout_interval,
                         result):
        for postact in self.postactions:
            if postact.command == "result":
#	    print "POSTACT: " + str(postact.command) + " " + str(result)
#           if postact.command == result:
                self.send(content = Content(timestamp = timestamp,
                                            command = result,
                                            tag = postact.tag,
                                            action = postact.tag,
                                            agent = self.agent),
                          timestamp = timestamp,
                          timeout_interval = timeout_interval)
            else:
                self.send(content = Content(timestamp = timestamp,
                                            command = postact.command,
                                            tag = postact.tag,
                                            action = postact.tag,
                                            agent = self.agent),
                          timestamp = timestamp,
                          timeout_interval = timeout_interval)
                
    # When triggered, fire the object (Generic behavior)
    def trigger(self):
        return self.fire(current_time())

    
        
#                                                           