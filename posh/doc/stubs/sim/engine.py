"""sim.engine Stubs

based on MASON documentation.
"""

from java.lang import Object

class AsynchronousSteppable(Object):
    pass

class MultiStep(Object):
    pass

class Schedule(Object):
    pass

class Sequence(Object):
    pass

class ParallelSequence(Sequence):
    pass

class RandomSequence(Sequence):
    pass

class SimState(Object):
    pass

class TentativeStep(Object):
    pass

class WeakStep(Object):
    pass

# Interfaces

class MakeSimState:
    pass

class Steppable:
    pass

class Stoppable:
    pass

class Asynchronous(Steppable, Stoppable):
    pass


