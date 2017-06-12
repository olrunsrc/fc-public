"""sim.portrayal.simple Stubs

based on MASON documentation.
"""

from java.lang import Object
from sim.portrayal import SimplePortrayal2D

class CircledPortrayal2D(SimplePortrayal2D):
    pass

class HexagonalPortrayal2D(SimplePortrayal2D):
    pass

class LabelledPortrayal2D(SimplePortrayal2D):
    pass

class OrientedPortrayal2D(SimplePortrayal2D):
    pass

class RectanglePortrayal2D(SimplePortrayal2D):
    pass

class ImagePortrayal2D(RectanglePortrayal2D):
    pass

class ValuePortrayal2D(RectanglePortrayal2D):
    pass

class ShapePortrayal2D(SimplePortrayal2D):
    pass

class TransformedPortrayal(SimplePortrayal2D):
    pass

# the filters are missing
