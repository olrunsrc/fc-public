from __future__ import nested_scopes

from basic import Element

class Competence_Element(Element):
    
    def __init__(self,
                 ce_label = "fix_me_cel",
                 action = None,
                 retries = -1,
                 competence = "not_my_competence",
                 **kw):
        # Call ancestor init with the remaining keywords
        Element.__init__(self, **kw)
        self.ce_label = ce_label
        self.action = action
        self.retries = retries
        self.competence = competence

    # Determine if the element is ready by first cheacking the
    # preconditions the action object
    def ready(self):
        for this_precon in self.action.preconditions:
            if not self.agent.blackboard.find_prereq(command = \
                                                     this_precon.command,
                                                     tag = this_precon.tag):
              
                return 0
        if self.trigger:
	    

            return self.trigger.trigger()

        else:
            return 1
