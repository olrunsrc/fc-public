"""Description of MASON Classes.

This module describes the API of the provided MASON classes that
are implemented in java. The implementation can be found in the 
same path as this file, but with the file endings .java.

The people working on the java implementations should make sure
to document any API change in this file to allow of a
consistent documentation.
"""
from __future__ import nested_scopes

# MASON
from sim.util import Double2D
from sim.engine import SimState
from sim.field.continuous import Continuous2D
from sim.display import GUIState, Console
from sim.portrayal import Oriented2D, Inspector
from sim.portrayal.simple import SimplePortrayal2D, OrientedPortrayal2D

# Java colours
from java.awt import Color

class Entity:
    """A MASON entity that features a location and is located in some fields.
    
    An Entity is the simplest element in a simulation. Its only properties are
    its location and it being part of some fields.
    
    To change the appearance of an Entity in a simulation, its method
    L{getPortrayal} has to be overridden.
    """
    def __init__( self, sim, fields, 
                 location = None, neighbourhoodSize = 40 ):
        """Initialises the entity and puts it on the given fields.
        
        The fields have to be given as a sequence of strings.
        The first fields that is given is interpreted as the default field.
        
        If no initial location is given, the location is initialised randomly.
        
        @param sim: The simulation state for the entity.
        @type sim: L{Simulation2D}
        @param fields: The fields that the entity is located in.
        @type fields: sequence of strings
        @param location: The initial location of the entity.
        @type location: L{Double2D}
        @param neighbourhoodSize: The initial size of the neighbourhood.
        @type neighbourhoodSize: Float
        """
        pass
    
    def getLoc( self ):
        """Returns the entity's current location.
        
        @return: The entity's current location.
        @rtype: L{Double2D}
        """
        pass
    
    def setLoc( self, loc ):
        """Sets the entity's location to the given location.
        
        @param loc: The new location of the agent.
        @type loc: L{Double2D}
        """
        pass
    
    def getRandomLocation( self ):
        """Returns a random location in the environment.
        
        The location is a samble from a uniform distribution over both
        coordinates.
        
        @return: Random location.
        @rtype: L{Double2D}
        """
        pass
    
    def randomiseLocation( self ):
        """Changes the location to a random position.
        
        The new location is determined by L{getRandomLocation}.
        """
        pass
        
    def getNeighbourhoodSize( self ):
        """Returns the current size of the neighbourhood.
        
        @return: Size of the neighbourhood.
        @rtype: Float
        """
        pass
    
    def setNeighbourhoodSize( self, neighbourhoodSize ):
        """Sets the size of the neighbourhood.
        
        If this size is smaller or equal to zero, it is
        ignored and the size of the neighbourhood remains
        unchanged.
        
        @param neighbourhoodSize: New size of the neighbourhood.
        @type neighbourhoodSize: Float
        """
        pass
    
    def getFields( self, fields = None ):
        """Returns the field objects for the given names.
        
        @param fields: The field names. If none are given, the method returns
            all fields that the entity is located in. Otherwise, the fields
            can either be given as a single string (for one single field) or
            as a sequence of string. Each string gives the name of a field.
        @type fields: None, string or sequence of strings
        @return: A sequence of fields.
        @rtype: tuple of L{Continuous2D<sim.field.continuous.Continuous2D>}
            objects
        """
        pass
    
    def distance( self, entityOrLocation ):
        """Returns the entity's distance to the given entity or location.
        
        @param entityOrLocation: If an entity is given, the distance to
            the entity is returned. In case of a given location, the
            distance to that location is returned. An entity is
            identified by checking for existence of the C{loc}
            attribute.
        @type entityOrLocation: L{Entity} or L{Double2D}
        @return: Distance.
        @rtype: Float
        """
        pass
    
    def vectorTowards( self, entityOrLocation ):
        """Returns a vector that points towards the given location/entity.
        
        @param entityOrLocation: The location/entity in question.
        @type entityOrLocation: L{Double2D} or L{Double2D}
        @return: Vector from agent location to given location.
        @rtype: L{Double2D}
        """
        pass
    
    def neighbours( self, fields = None, neighbourhoodSize = 0.0 ):
        """Returns all entities in the current neighbourhood.
        
        This method returns all objects on the given fields within
        at least the set neighbourhood. For a more detailed documentation
        on how this method operates, please have a look at the MASON
        documentation of C{sim.fields.continuous.Continuous2D.getObjectsWithinDistance}.
        The neighbourhood can be accessed by L{getNeighbourhoodSize} and
        L{setNeighbourhoodSize}.
        
        If a neighbourhoodSize is given, then this size of neighbourhood is
        considered, rather than the currently set neighbourhood.
        
        The neigbourhood never contains the entitiy it is called for.
        
        @param fields: As for the L{getFields} method.
        @type fields: None, string or sequence of strings
        @param neighbourhoodSize: If given, determines the size of the neighbourhood.
        @type neighbourhoodSize: Float
        @return: The entities in the neighbourhood.
        @rtype: sequence of entities
        """
        pass
    
    def closest( self, fields = None, exclude = None ):
        """Returns the closest other entity in all given fields.
        
        The method allows for exclusion of a set of entities from
        the search for the closest other entitiy. The entity for which
        the method is called is excluded by default.
        
        If no other entity is found, None is returned.
        
        @param fields: As for the L{getFields} method.
        @type fields: None, string of sequence of strings
        @param exclude: A sequence of entites to exclude from the
            search for the closest other entity.
        @type exclude: Sequence of L{Entity} objects
        @return: The closes other entity.
        @rtype: None or L{Entity}
        """
        pass
    
    def fieldWidth( self, fields = None ):
        """Returns the maximum distance between any two entities
        in the given fields.
       
        Note that this method is computationally expensive as it has to
        compare any entity with any other entity. O(n^2).
        
        @param fields: As for the L{getFields} method.
        @type fields: None, string or sequence of strings
        @return: The maximum distance between any two entities.
        @rtype: Float
        """
        pass
    
    def averageFieldWidth( self, fields = None ):
        """Returns the average distance between any two entities
        in the given fields.
        
        If several of the given fields contain the same etities, then
        the value returned by that method will be wrong.
        
        Note that this method is computationally expensive as it has to
        compare any entity with any other entity. O(n^2).
        
        @param fields: As for the L{getFields} method.
        @type fields: None, string or sequence of strings
        @return: Average distance between any two entities.
        @rtype: Float
        """
        pass

    def fieldCentre( self, fields = None ):
        """Returns the centre point between between the two entities which
        are furthest apart in the given field.
        
        If no (other) entities are found, the entity's location is returned.

        Note that this method is computationally expensive as it has to
        compare any entity with any other entity. O(n^2).
        
        @param fields: As for the L{getFields} method.
        @type fields: None, string or sequence of strings
        @return: Centre point of the field.
        @rtype: L{Double2D}
        """
        pass
    
    def distanceToCentre( self, fields = None ):
        """Returns the entity's distance to centre.
         
         The centre is computed by L{fieldCentre}.
         
         @param fields: As for the L{getFields} method.
         @type fields: None, string of sequence of strings
         @return: Distance from field's center.
         @rtype: Float
        """
        pass

    def getPortrayal( self ):
        """Returns a portrayal instance that determines how the
        entity is drawn on the GUI.
        
        If the appearence of the entity is to be customised, this
        method has to be overridden. See the MASON documentation for the
        class SimplePortrayal2D to get a list of the available options.
        
        @return: The portrayal for the entitiy.
        @rtype: Descendent of L{SimplePortrayal2D}
        """
        pass


class OrientedEntity( Entity, Oriented2D ):
    """A MASON agent based on L{Entity} with an additional orientation.
    
    It implements the interface of L{Oriented2D} to show its orientation in
    the simulation GUI.
    
    The orientation can either be set by the user, or is determined when
    changing the location of the agent. As the agent is aware of its orientation,
    it can move forward a certain distance without having to speciy the target
    location.
    """
    def __init__( self, sim, fields, location = None, orientation = None ):
        """Initialises the agent.
        
        See L{Entity} for documentation of the arguments.
        
        If no initial orientation is given, the agent's orientation
        is initialised randomly.
        
        @param orientation: The initial orientation of the agent in degrees.
        @type orientation: Float
        """
        pass
    
    def setLoc( self, loc ):
        """Sets the agent's location and corrects its orientation.
        
        The agent's location is set of all the fields it is located on.
        
        Its orientation is set to the direction from the previous
        location to the new location, to reflect the direction that
        the agent is moving in.
        
        @param loc: The new location.
        @type loc: L{Double2D}
        """
        pass

    def orientation2D( self ):
        """Returns the current orientation of the agent, in radians.
        
        This method is required by the class OrientedPortrayal2D to display
        the agent correctly.
        
        @return: Agent's orientation in radians.
        @rtype: Float
        """
        pass
    
    def getOrientation( self ):
        """Returns the current orientation of the agent, in degrees.
        
        @return: Agent's orientation in degrees.
        @rtype: Float
        """
        pass
    
    def setOrientation( self, orientation ):
        """Sets the agent's orientation.
        
        @param orientation: New orientation in degrees.
        @type orientation: Float
        """
        pass
    
    def randomiseOrientation( self ):
        """Sets the orientation to some random value.
        """
        pass
    
    def setDirection( self, entityOrLocation ):
        """Sets the agent's orientation to point towards the given location.
        
        @param entityOrLocation: The location or entity to look towards.
        @type entityOrLocation: L{Double2D}
        """
        pass
    
    def move( self, distance ):
        """Moves a certain distance in the agent's current direction,
        determined by its orientation.
        
        @param distance: The distance to move.
        @type distance: Float
        """
        pass
    
    def moveTo( self, entityOrLocation, distance ):
        """Moves the agent the given distance towards the given location.
        
        This method is a combination of L{setDirection} and L{move}.
        
        @param entityOrLocation: The location/entity to move towards.
        @type entityOrLocation: L{Double2D}
        @param distance: The distance to move.
        @type distance: Float
        """
        pass
    
    def moveBy( self, x, y ):
        """Moves the agent by (x, y) from the current location.
        
        @param x: The movement in x.
        @type x: Float
        @param y: The movement in y.
        @type y: Float
        """
        pass
        
    def getPortrayal( self ):
        """Returns a white oriented portrayal.
        
        @return: An oriented portrayal.
        @rtype: L{OrientedPortrayal2D}
        """
        pass


class Simulation2D( SimState ):
    """A MASON simulation state.
    
    As simulation state represents the current state of a simulation,
    and therefore contains all elements of the simulation (e.g. fields,
    agents, entities).
    
    This class represents simulations in a 2D continuous toroidal environment
    with several layerd fields. An agent/entity can belong to several fields
    at once. The simulation schedule is a standard single-order synchronous
    stepped schedule, i.e. each agent is update once per time step of the
    simulation.
    """
    def __init__( self, width, height, fieldNames, randomSeed = None ):
        """Initialises the MASON simulation.
        
        The dimensions of the continuous toroidal environment are specified by
        the C{width} and C{height} arguments. The fields with the given names
        are created as L{Continuous2D} fields of the same size as the environments.
        
        The schedule is set to a standard single-order schedule. If no random seed
        is given, then the random number generator is initialised with the
        default seed.
        
        @param width: Width of the environment.
        @type width: Float
        @param height: Height of the environment.
        @type height: Float
        @param fieldNames: The names of the fields to create.
        @type fieldNames: sequence of strings
        @param randomSeed: Seed for the random number generator.
        @type randomSeed: Integer
        """
        pass
    
    def start( self ):
        """Called immediately prior to starting the simulation,
        or in-between simulation runs.
        
        This method is called by the MASON simulation control.
        
        It calles L{addEntities} which needs to be overriden by
        the simulation.
        """
        pass
    
    def addEntities( self ):
        """Adds agents and entities to the simulation.
        
        This method raises an exception by default, and needs to be overridden
        to add all agents and entities to the simulation. By default,
        all fields are empty before this method is called.
        
        @raise NotImplementedError: Always
        """
        pass
    
    def getField( self, fieldName ):
        """Returns the field object with the given name.
        
        @param fieldName: A field name.
        @type fieldName: string
        @return: The field object associated with the given field name.
        @rtype: L{Continuous2D}
        """
        pass
    
    def getFields( self, fieldNames ):
        """Returns a tuple with the field objects of the given names.
        
        @param fieldNames: A sequence of field names.
        @type fieldNames: Sequence of strings
        @return: A sequence of field objects.
        @rtype: Tuple of L{Continuous2D}
        """
        pass
    
    def createFields( self, fieldNames ):
        """Adds fields with given names to the simulation.
        
        @param fieldNames: The names of the fields. They can either be
            given as a single string (in which case a single field is
            added), or as a sequence of strings.
        @type fieldNames: single string or sequence of strings
        """
        pass


class Simulation2DGUI( GUIState ):
    """The GUI for a simulation.
    
    The class takes a simulation state of class L{Simulation2D} and creates
    a GUI for it that displays the environment and allows control
    of the simulation.
    
    The environment is displayed in a single frame, and shows all of
    the given fields. Control is provided by the standard MASON
    controller.
    """
    def __init__( self, simulation, name, maxSize, portrayalFields ):
        """Initialises the simulation GUI.
        
        The maximum size argument gives the maximum siye of either the width
        or the height of the environment display. The real size of the
        simulation environment does not have to correspond to the maximum
        size. It only determined how large the environment is displayed.
        
        The portrayal fields are all fields that will be shown in the
        environment. This list of fields has to contain any field that
        is the default field of an agent or entity in the simulation. The
        GUI only displays agents / entities that are located in the given
        fields. The layering of the field agents / entities is determined
        by the ordering of the fields in the list. Agents / entities in
        earlier fields are drawn over agents / entites in later fields.
        
        For example, if we have two fields 'agents' and 'entities', and
        the agents should appear on top of the entities, the the
        portrayalFields has to be given in the order ('agents', 'entities').
        
        @param simulation: The simulation to show.
        @type simulation: L{Simulation2D}
        @param name: The name of the simulation. This name will be the
            frame title of the simulation control frame.
        @type name: string
        @param maxSize: The maximum width or height of the simulation
            environment frame.
        @type maxSize: Integer
        @param portrayalFields: The names of the fields to show in the
            simulation.
        @type portrayalFields: Sequence of strings
        """
        pass
    
    def getName( self ):
        """Returns the name of the simulation.
        
        This method is called by the MASON simulation control to get
        the name of the control frame.
        
        @return: Name of the simulation.
        @rtype: string
        """
        pass
    
    def init( self, controller ):
        """Called to initialize (display) windows etc.
        
        This method is called by the MASON simulation control.
        
        It calls L{setupDisplay} to set up the simulation environment frame,
        and attaches the fields to environment display.        
        For each declared field, it calls getPortrayal() of all
        agents and entities to set how they are displayed.

        
        @param controller: A MASON simulation controller.
        @type controller: L{sim.display.Controller}
        """
        pass
    
    def setupDisplay( self, controller ):
        """Sets up the display to show the simulation environment.
        
        In this implemetation, it calls L{createDisplay} to create
        a frame with black background and with a title equal to the name
        of the simulation. To change this, this method has to be
        overridden.
        
        @param controller: A MASON simulation controller.
        @type controller: L{sim.display.Controller}
        """
        pass
    
    def createDisplay( self, controller, title, bgcolour ):
        """Creates the simulation environment display.
        
        @param controller: A MASON simulation controller.
        @type controller: L{sim.display.Controller}
        @param title: The title of the display frame.
        @type title: string
        @param bgcolour: The background colour of the simulation
            environment.
        @type bgcolour: Java L{Color}
        """
        pass
    
    def start( self ):
        """Called immediately prior to starting the simulation,
        or in-between simulation runs.
        
        This method is called by the MASON simulation control.
        
        It calls L{setupPortrayals} to initialise the portayals of
        the agents and entities.
        """
        pass
    
    def setupPortrayals( self ):
        """Sets up the portrayals of all entities and agents.
        
        This method calls L{Entity.getPortrayal} for each entity
        and agent on all fields that will be displayed, to set up
        how the entities and agents are displayed.
        """
        pass
    
    def load( self ):
        """Called by the Console when the user is loading in a new
        state from a checkpoint.
        
        The method calls L{setupPortrayals}.
        """
        pass

    def quit( self ):
        """Called by the Console when the user is quitting the SimState.
        """
        pass
    
    def run( self ):
        """Creates a controller and gives control to it.
        """
        pass