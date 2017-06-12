"""sim.portrayal Stubs

based on MASON documentation.
"""

from java.lang import Object
from javax.swing import JPanel

class Inspector(JPanel):
    pass

class SimpleInspector(Inspector):
    pass

class DrawInfo2D(Object):
    pass

class HexaDrawInfo2D(DrawInfo2D):
    pass

class FieldPortrayal(Object):
    pass

class FieldPortrayal2D(Object):
    pass

class LocationWrapper(Object):
    pass

class SimplePortrayal2D(Object):
    pass

# Interfaces

class Oriented2D:
    pass

class Portrayal:
    pass

class Portrayal2D(Portrayal):
    pass
