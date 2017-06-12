from __future__ import nested_scopes

#import basic
#import generic
#import complex
#import action_pattern
#import blackboard
#import competence
#import competence_element
#import drive_collection
#import drive_element
#import messages
#import schedule
#import sense

import basic,generic,complex,action_pattern,blackboard,competence,competence_element,drive_collection,drive_element,messages,schedule,sense 
from generic import *
class Sense(Generic):
    
    def __init__(self,
                 sensor = None,
                 sense_value = None,
                 sense_predicate = None,
                 **kw):
        # Call ancestor init with the remaining keywords
        Generic.__init__(self, **kw)
        self.sensor = sensor
        self.sense_value = sense_value
        self.sense_predicate = sense_predicate
        # The sensor is called when sensep need to construct a comparison
        # using eval(self.sensor() comp_operator value) where
        # comp_operator is the predicate(eq, lt, gt, ne).
        
    def fire(self, timestamp, timeout_interval = DEFAULT_VIEWTIME):
        result = self.sensep()
        if result:
            self.fire_postactions(timestamp = timestamp,
                                  timeout_interval = timeout_interval,
                                  result = "done")
        else:
            self.fire_postactions(timestamp = timestamp,
                                   timeout_interval = timeout_interval,
                                   result = "fail")
        return result
    
    def sensep(self):
        if not self.sense_value:
            return self.sensor()
        else:
            # print "eval('self.sensor() " + self.sense_predicate + " " + \
            #            self.sense_value + "')"
            # This is touchy. Due to psyco, we need to first run the
            # sensor, and then evaluate it.
            # return eval("self.sensor() " + self.sense_predicate + " " + \
            #             self.sense_value)
            result = self.sensor()
            return eval(repr(result) + " " + self.sense_predicate + " " + \
                        self.sense_value)
