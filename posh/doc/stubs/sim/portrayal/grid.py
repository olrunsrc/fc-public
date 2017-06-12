"""sim.portrayal.grid Stubs

based on MASON documentation.
"""

from sim.portrayal import FieldPortrayal2D

class ObjectGridPortrayal2D(FieldPortrayal2D):
    pass

class FastObjectGridPortrayal2D(ObjectGridPortrayal2D):
    pass

class HexaObjectGridPortrayal2D(ObjectGridPortrayal2D):
    pass

class FastHexaObjectGridPortrayal2D(HexaObjectGridPortrayal2D):
    pass

class SparseGridPortrayal2D(FieldPortrayal2D):
    pass

class HexaSparseGridPortrayal(SparseGridPortrayal2D):
    pass

class ValueGridPortrayal2D(FieldPortrayal2D):
    pass

class FastValueGridPortrayal2D(ValueGridPortrayal2D):
    pass

class HexaValueGridPortrayal2D(ValueGridPortrayal2D):
    pass

class FastHexaValueGridPortrayal2D(HexaValueGridPortrayal2D):
    pass
