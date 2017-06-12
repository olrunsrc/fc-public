from __future__ import nested_scopes


from generic import *


class Complex(Generic):
    
    def __init__(self,
                 elements = [],
                 active = None,
                 **kw):
        # Call ancestor init with the remaining keywords
        Generic.__init__(self, **kw)
        self.elements = elements
        self.active = active
        
    def schedule_element(self,
                         element,
                         preconditions,
                         postactions,
                         timestamp):
        if (type(element) is MethodType) or (type(element) is FunctionType):
            # If the element is a Method or Function then make a new
            # Generic object to encapsulate it and put it on the schedule
            self.agent.schedule.add_item(
                Generic(name = self.tag,
                        command = "request",
                        tag = self.tag,
                        action = element,
                        value = self.value,
                        timestamp = timestamp,
                        drive_name = self.drive_name,
                        preconditions = preconditions,
                        postactions = postactions,
                        timeout = timestamp + self.timeout_interval,
                        timeout_interval = self.timeout_interval,
                        agent = self.agent))
        elif (type(element) is InstanceType):
            # If the element is a object instance of a Class, we
            # need to check which class it derives from
            if isinstance(element, Competence_Element):
                self.agent.schedule.add_item(
                    element.action.make_instance(drive_name = self.drive_name,
                                                 timestamp = timestamp,
                                                 preconditions = preconditions,
                                                 postactions = postactions))
            elif isinstance(element, Generic):
                self.agent.schedule.add_item(
                    element.make_instance(drive_name = self.drive_name,
                                          timestamp = timestamp,
                                          preconditions = preconditions,
                                          postactions = postactions))
            else:
                # Error: Instance Invaild
                pass
        else:
            # Error: Object Invalid
            pass
            