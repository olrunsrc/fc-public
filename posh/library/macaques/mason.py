"""The MASON behaviour module and classes to initialise the MASON simulation and
GUI.

Overview
========

This module provides several classes that ease the simulation of POSH
agents in a continuous 2D MASON environment. The MASON behaviour class
establishes communication between the agent and the MASON simulation.
The MASONWorld class sets up the simulation itself and the GUI for the
simulation.

The Simulation Environment
==========================

The Concept of 'Fields'
-----------------------

The basic elements of a simulation are the environment, and the objects
in the environment. The environment is a continuous toroidal plane of
a certain width and height, and is defined by a set of layered 'field'.
Each field covers the whole environment and may contain agents and
entities. Usually, one field contains entities and agents that are in
some relation to each other. The use of fields is to allow agents to
restrict certain functionality (like determining which is the closest
agent to onesself) to a particular set of fields.

Given, for example, that we have a set of male and female agents. Let
us create the fields 'male', 'female' and 'agents', and let all male agents
be located in the fields 'male' and 'agents' (entities and agents can be
located in several fields at once), and all female agents be located in
the fields 'female' and 'agents'. If any agent would now want to know
which is the closest other agent, independent of the gender, it would
use C{closest('agents')} to look for the closest agent in the 'agents'
field. Would an agent want to find the closest male agent, it would
use C{closest('male')} to restrict the search to the 'male' field.
Even though this example only deals with the method L{Entity.closest},
the same concept can be applied to any method that takes a set of
fields as an argument (as many of the L{Entity} class do).

The Simulation
--------------

The MASON simulation is set up through the L{MASONWorld} class. Even though
MASON conceptually separates the simulation from its visualisation, this
separation is not considered by L{MASONWorld} which initialises both the
simulation and its GUI.

Setting up a simulation is as easy as calling the constructor of L{MASONWorld}
and then invoking the run() method of the created instance. Firsly, this causes
only the simulation and the GUI to be set up, together with the standard
MASON controller to control the steps of the simulation. Every time the
simulation is started or restarted by the user, all agents are created and put
into the environment. Thus, the simulation always starts the the same initial
conditions that are given to the constructor of L{MASONWorld}.

Using the MASON Behaviour
-------------------------

The MASON behaviour links the agents to the MASON simulation and provides an
interface for the agents to act and sense in this simulation. Thus, changes to
the agent's state within the environment have to be performed using the methods
of the MASON behaviour. The MASON behaviour also inherits the methods of the
L{OrientedEntity} and the L{Entity} class, and therefore also its methods.
The documentation provides a L{summary<MASON>} of its
API.

Control of the agent is also performed through the MASON behaviour. Every
instance of this behaviour also keeps an instance of L{_MASONControl} that
reacts to the control of the simulation. At every step of this simulation,
the control runs one loop of the POSH plan and hence allows the agent to act
and react in the environment.

How an agent is visualised in the simulation GUI is controlled by its
'portrayal', which can be set by the L{setPortrayal<MASON.setPortrayal>} method
of the behaviour, or by its C{display} attribute, with the same syntax as for
L{setPortrayal<MASON.setPortrayal>}.

For the attributes that are considered by L{MASON} behaviour, please consult
its API L{documentation<MASON>}.
"""

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# for time.sleep()
import time
import re

# Java colours, layout, and event handler, GUI components
from java.awt import Color, Dimension
from java.awt.event import ActionListener
from javax.swing import JTextField, JLabel, BoxLayout, Box

# MASON Java stuff
from sim.engine import Steppable
from sim.util import Double2D
from sim.util.gui import LabelledList
from sim.display import Console
from sim.portrayal import SimplePortrayal2D, Inspector
from sim.portrayal.simple import LabelledPortrayal2D, \
    CircledPortrayal2D, OrientedPortrayal2D
# If this module is loaded by epydoc, then we provide the documentation stubs
# rather than the Java classes
if is_jython:
    from platform_files.MASON import Simulation2D, Simulation2DGUI, \
        OrientedEntity
else:
    from platform_files.MASON.mason_doc import Simulation2D, Simulation2DGUI, \
        OrientedEntity, Entity

# POSH Behaviour
from POSH import Behaviour, create_agents


class _MASONControl(Steppable):
    """The MASON interface that controls the POSH agent.
    
    To avoid cluttering the agent namespace, this class is instantiated in
    every MASON behaviour to control the agent from the MASON scheduler.
    
    It provides an L{Steppable} MASON agent that gets its L{step()} method called
    at every step of the simulation. The object then calls
    L{followDrive<POSH.AgentBase.followDrive>} of the associated POSH agent to
    update the agent's behaviour.
    """
    def __init__(self, agent):
        """Initialises the agent control, and sets the simulation schedule.
        
        The schedule is set to perform one step at each time step.
        
        @param agent: The agent that is controlled.
        @type agent: L{POSH.AgentBase}
        """
        self.agent = agent
        self.agent.getWorld().sim.schedule.scheduleRepeating( self )
    
    def step(self, state):
        """Performs one step in the simulation.
        
        This method is called by the MASON simulation scheduler.
        
        @param state: The state of the simulation.
        @type state: L{Simulation2D}
        """
        self.agent.followDrive()


class _BehaviourInspector( Inspector ):
    """An inspector to show/modify the inspectors provided by the behaviours.
    
    This class provides an inspector GUI to the behaviour inspectors.
    It builds the inspectors by iterating through the list of
    behaviours and querying each of them for their inspectors.
    The values are given by JTextField's if they can be modified, and by
    JLabel's otherwise.
    
    TODO: There is a nifty feature in newer MASON version to plot a graph of
    the values that are inspected. This is achieved by adding a pop-down menu
    that is added via the sim.portrayal.inspector.PropertyInspector.getPopupMenu
    method. Unfortunately, this class requires the given properties to be
    of class sim.utils.Properties. This can easily be achieved by wrapping the
    inspectors for a given behaviour by a class that inherits
    sim.utils.Properties. This also allows the use of sim.util.gui.PropertyField
    rather than the currently used JTextField to modify the values. More details
    on how to implement such a thing can be found by having a distant look
    at the source sim.portrayal.SimpleInspector, particluarly its method
    generateProperties() where the properties are extracted and added to the
    GUI (using a LabelledList), and its method makePropertyField(), where the
    property text field (or however it appears) is generated. The MASON API doc
    also helps a lot.
    """
    def __init__(self, wrapper):
        """Initialises the behaviour inspector.
        
        @param wrapper: The object wrapper given to the portrayal.
        @type wrapper: sim.portrayal.LocationWrapper
        """
        Inspector.__init__(self)
        # getObject() returns the MASON behaviour -> .agent to access agent
        agent = wrapper.getObject().agent
        # get list of behaviours and sort them by name
        beh_list = agent.getBehaviours()
        beh_list.sort(
            lambda x, y: cmp(x.getName().upper(), y.getName().upper()))
        beh_names = []
        for beh in beh_list:
            beh_names.append(beh.getName())
        self.setLayout(BoxLayout(self, BoxLayout.PAGE_AXIS))
        # add agent information
        agentList = LabelledList("Agent")
        agentList.addLabelled("Name ", JLabel(agent.MASON.getAgentName()))
        agentList.addLabelled("Behaviours ", JLabel(", ".join(beh_names)))
        # need to restrict maximum size to allow for glue at end of panel to
        # expand
        agentList.setMaximumSize(
            Dimension(int(agentList.getMaximumSize().getWidth()),
                int(agentList.getMinimumSize().getHeight())))
        self.add(agentList)
        # build the behaviour inspectors one by one, and collect their
        # properties
        self._properties = []
        for beh in beh_list:
            self._properties.extend(self._buildInspectorGUI(beh))
        # give message if no inspectors defined
        if not self._properties:
            self.add(JLabel("No inspectors defined!"))
        self.add(Box.createVerticalGlue())
        
    def updateInspector(self):
        """Updates the inspector.
        """
        for property in self._properties:
            # property is (Component, accessor)
            property[0].setText(str(property[1]()))
    
    def _buildInspectorGUI(self, beh):
        """Builds the inspector GUI for the given behaviour object.
        
        @param beh: The behaviour object.
        @type beh: C{Behaviour}
        """
        inspectors = beh.getInspectors()
        # ignore behaviours that don't provide any inspectors
        if not inspectors:
            return []
        
        # class to modify the behaviour state from the GUI
        class UpdateBehaviourState(ActionListener):
            def __init__(self, modifier, component):
                self._modifier = modifier
                self._component = component
            def actionPerformed(self, evt):
                self._modifier(str(self._component.getText()))
        
        # build the list
        propertyList = LabelledList(beh.getName())
        properties = []
        for inspector in inspectors:
            # create textfield if mutator is available
            component = None
            state_str = str(inspector[1]())
            if inspector[2]:
                component = JTextField(state_str)
                # react on changes of values by the user
                component.addActionListener(
                    UpdateBehaviourState(inspector[2], component))
            else:
                component = JLabel(state_str)
            properties.append((component, inspector[1]))
            propertyList.addLabelled("%s " % inspector[0], component)
            # Needs to create an actual list of properties of class Properties
            #propertyList.add(None,
            #                 JLabel("%s " % inspector[0]),
            #                 PropertyInspector(properties, index, guistate),
            #                 component,
            #                 None)
        # add it to the GUI and return the list of properties with its
        # accessors and components
        # need to restrict maximum size to allow for glue at end of panel to
        # expand
        propertyList.setMaximumSize(
            Dimension(int(propertyList.getMaximumSize().getWidth()),
                int(propertyList.getMinimumSize().getHeight())))
        self.add(propertyList)
        return properties


class MASON(Behaviour, OrientedEntity):
    """The MASON behaviour that links the agent to the MASON environment.
    
    The behaviour does not by itself provide any actions or senses, but allows
    other behaviours through the use of its method access to control the
    agent and sense its environment.
    
    The behaviour considers the following attributes:
        - C{fieldNames} : puts the name on the fields with the given names.
          the names need to be given as a string 'field1, field2, ...'
        - C{display} : sets how the agent is displayed in the GUI. This is
          specified by a string that has the same syntax as the one
          given to L{setPortrayal}.
        - C{name} : name of the agent how it is shown in the GUI. This name
          is given by a string.
    """
    # matches 'number-number-number'
    _rgbcolour_match = re.compile(\
        r'^[ \t]*\d{0,3}[ \t]*-[ \t]*\d{0,3}[ \t]*-[ \t]*\d{0,3}[ \t]*$')
    def __init__(self, agent):
        Behaviour.__init__(self, agent, (), ())
        # initially put the agent on all fields, as we might assign
        # the attribute self.fields later
        OrientedEntity.__init__(self, agent.getWorld().sim)
        # add controller to control the agent
        self._control = _MASONControl(self.agent)
        # sets the MASON random number generator
        self._setMASONRNG(self.agent.getWorld().sim)
        # default portrayal: white compass of size 3
        self._portrayal = OrientedPortrayal2D(SimplePortrayal2D(),
                              0, 3.0, Color.white,
                              OrientedPortrayal2D.SHAPE_COMPASS)
        # standard attributes (should be changed after initialisation)
        self.fieldNames = None
        self.display = None
        self.name = 'noname'
    
    def reset(self):
        """Considers the attributes that were assigned to the behaviour after
        construction.
        
        The attributes that are considered are documented in the L{MASON} class
        documentation.
        """
        # set fields that the agent resides in
        if self.fieldNames != None:
            # fields can be either a sequence of strings, or 'field1, field2, ..'
            if type(self.fieldNames) == type(''):
                fields = [field_name.strip() \
                          for field_name in self.fieldNames.split(',')]
            else:
                fields = self.fieldNames
            self.setFields(fields)
        # set portrayal
        if self.display != None:
            portrayal_info = [display_part.strip() for \
                              display_part in self.display.split(',')]
            self.setPortrayal(portrayal_info)
        return True
    
    def getAgentName(self):
        """Returns the name of the agent for the inspector window.
        
        This name is determine by the 'name' attribute of the MASON behaviour.
        
        @return: name of the agent
        @rtype: string.
        """
        return self.name

    def assignAttributes(self, attributes):
        """Assigns the behaviour a set of attributes.
        
        This method picks out the attribute 'loc' and uses it to set to
        location of the agent. All other attributes are assigned by
        L{Behaviour.assignAttributes}. The location has to be given by
        C{MASON.loc = 100.0, 200.0}, where the value is to be given as a
        string ('100.0, 200.0' in this example), and the first value and the
        second value specify the x and the y coordinate respectively.
        
        @param attributes: dictionary of attributes to assign to behaviour.
        @type attributes: dictionary attribute_name -> value
        """
        # filter out 'loc' to set location of agent using setLoc
        if attributes.has_key('loc'):
            x, y = map(float, attributes['loc'].split(','))
            self.setLoc(Double2D(x, y))
            del attributes['loc']
        Behaviour.assignAttributes(self, attributes)

    def setPortrayal(self, portrayal_info):
        """Sets the portrayal of the agent.

        The portrayal_info is given as a sequence [shape, colour, size,
        [name]], where the last element is optional. Each of the elements
        of the sequence is given as a string.

        The shape can be one of the following: C{'compass'} - a
        compass-shaped portrayal. C{'kite'} - a kite-shaped portrayal.
        C{'circle'} - a cirlce with a line that gives the orientation.
        C{'line'} - a simple line (might not be very visible).

        The color can be one of the following: C{'black'}, C{'blue'},
        C{'cyan'}, C{'darkgray'}, C{'gray'}, C{'lightgray'}, C{'green'},
        C{'magenta'}, C{'orange'}, C{'pink'}, C{'red'}, C{'white'},
        C{'yellow'}. Alternatively, the colour can be given as the RGB
        values (from 0 to 255) separated by a dash (e.g. C{'0-0-255'}
        for blue).

        The size is a floating-point number.

        If a name is given, then the portrayal gets this name attached.
        
        This method is automatically called from L{reset()} if the 'display'
        attribute of the behaviour is set.

        @param portrayal_info: The shape, colour, size and name of the
            agent portrayal.
        @type portrayal_info: sequence of 3 or 4 strings
        @raise SyntaxError: If one of the elements in the portrayal_info
            is not understood.
        """
        # FIXME: This can be moved into the OrientedEntity
        colour_map = { 'black' : Color.black, 'blue' : Color.blue,
                       'cyan' : Color.cyan, 'darkgray' : Color.darkGray,
                       'gray' : Color.gray, 'lightgray' : Color.lightGray,
                       'green' : Color.green, 'magenta' : Color.magenta,
                       'orange' : Color.orange, 'pink' : Color.pink,
                       'red' : Color.red, 'white' : Color.white,
                       'yellow' : Color.yellow }
        # check the syntax
        shape_str = portrayal_info[0].lower()
        if not shape_str in ['compass', 'kite', 'circle', 'line']:
            raise SyntaxError, "Portrayal type '%s' is invalid" % \
                  portrayal_info[0]
        if (not colour_map.has_key(portrayal_info[1].lower())) and \
           (not self._rgbcolour_match.match(portrayal_info[1])):
            raise SyntaxError, "'%s' is not a valid colour value" % \
                  portrayal_info[1]
        # get the colour either from the name of by digits
        if colour_map.has_key(portrayal_info[1].lower()):
            colour = colour_map[portrayal_info[1].lower()]
        else:
            colour_numbers = portrayal_info[1].split('-')
            r, g, b = int(colour_numbers[0]), int(colour_numbers[1]), \
                      int(colour_numbers[2])
            colour = Color(r, g, b)
        # create the different portrayals, based on shape
        size = float(portrayal_info[2])
        if shape_str == 'compass':
            portrayal = OrientedPortrayal2D(SimplePortrayal2D(), 0, size,
                                            colour,
                                            OrientedPortrayal2D.SHAPE_COMPASS)
        elif shape_str == 'kite':
            portrayal = OrientedPortrayal2D(SimplePortrayal2D(), 0, size,
                                            colour,
                                            OrientedPortrayal2D.SHAPE_KITE)
        elif shape_str == 'line':
            portrayal = OrientedPortrayal2D(SimplePortrayal2D(), 0, size,
                                            colour,
                                            OrientedPortrayal2D.SHAPE_LINE)
        else:
            portrayal = \
                CircledPortrayal2D( \
                    OrientedPortrayal2D(SimplePortrayal2D(), 0, size,
                        colour, OrientedPortrayal2D.SHAPE_LINE),
                    0, size, colour, 0)
        # add the name if given
        if len(portrayal_info) == 4:
            name = portrayal_info[3]
            portrayal = LabelledPortrayal2D(portrayal, name)
        self._portrayal = portrayal

    def getPortrayal(self):
        """Returns the portrayal that was set by L{setPortrayal}.

        If no portrayal has been set before, a white compass-shaped
        portrayal of size 3 is returned.

        This method is called upon initialising the agent and returns
        the portrayal object.  This method should not be called by the user
        of a mason agent. L{setPortrayal} is supposed to be used
        instead.
        """
        # FIXME: This can be (partially) moved into the OrientedEntity
        # class. To add inspectors
        # we're abusing labelled portrayal to create a portrayal without
        # an appearance that can modify the inspector. The label
        # will be empty (that's the trick ;-))
        class InspectorPortrayal( LabelledPortrayal2D ):
            def __init__(self, portrayal):
                LabelledPortrayal2D.__init__(self, portrayal, "")
            def getInspector(self, wrapper, guiState):
                return _BehaviourInspector(wrapper)
        # add the inspector to the portrayal
        return InspectorPortrayal(self._portrayal)
    
    def _setMASONRNG(self, simulation):
        """Assigns the MASON random number generator to the POSH agent.
        """
        # that's a bit of a hack, as jython sometimes behaves a bit nasty with
        # respect to passing classes inherited from java classes around. Hence,
        # we'll reflect the necessary methods into a python class
        class rng:
            def __init__(self, generator):
                self.random = generator.nextDouble
                self.randint = lambda a, b: a + generator.nextInt(b - a + 1)
                self.uniform = lambda a, b: a + (b - a) * generator.nextDouble()
                self.gauss = lambda mu, sigma: mu + sigma * generator.nextGaussian()
        # class POSH agent setRNG
        self.setRNG(rng(simulation.random))


class MASONWorld:
    """A MASON environment providing both the simulation and the GUI.
    
    In MASON the simulation is separate from the GUI, as it can be run without
    it. We do not provide this separation here, and provide both the simulation
    and the GUI at the same time.
    
    Upon construction of a MASONWorld instance, the simulation and the
    GUI are set up, but no agents are added. The agents are set up as soon as
    the start() method of the simulation is called (which is when the 'play'
    button on the MASON control is clicked).
    
    The agents are not removed when the 'stop' button is pushed, which
    calls the finish() method of the simulation. Instead, they are removed as
    soon as a new simulation is started.
    """
    def __init__(self, library, agents_init,
                 name, width, height, fields,
                 display_size = 750, background_colour = None):
        """Sets up a MASON simulation an GUI for the given library and agents.
        
        @param library: name of the behaviour library (usually 'mason')
        @type library: string
        @param agents_init: agents initialisation info structure
        @type agents_init: as returned by
            L{POSH.agentinitparser.parse_agent_init_file}
        @param name: Name of the simulation, as shown in the GUI
        @type name: string
        @param width: witdh of the simulation environment
        @type width: int
        @param height: height of the simulation environment
        @type height: int
        @param fields: fields that the agents can reside in
        @type fields: sequence of strings
        @param display_size: maximum width/height of environment display
        @type display_size: int
        @param background_colour: background colour of the environment display
        @type background_colour: L{java.awt.Color}
        """
        # set up simulation and its GUI
        self.sim = self._setupSim(width, height, fields)
        self.gui = self._setupGUI(name, fields, display_size, background_colour)
        # store the rest to set up the agents
        self._library = library
        self._agents_init = agents_init
    
    def _setupSim(self, width, height, fields):
        """Sets up the simulation environment.
        
        @param width: width of the simulation environment
        @type width: int
        @param height: height of the simulation environment
        @type height: int
        @param fields: fields that the agents can reside in
        @type fields: sequence of strings
        @return: the simulation instance
        @rtype: L{Simulation2D}
        """
        class CustomSim(Simulation2D):
            """Custom simulation class to react to addEntities request.
            """
            def __init__(self, width, height, fields, world):
                Simulation2D.__init__(self, width, height, fields)
                self._world = world
            
            def addEntities(self):
                self._world._initSimulation()
        
        return CustomSim(width, height, fields, self)
    
    def _setupGUI(self, name, fields, display_size, background_colour = None):
        """Sets up the simulation GUI.
        
        @param name: name of the simulation, as shown in the GUI
        @type name: string
        @param display_size: maximum width/height of environment display
        @type display_size: int
        @param background_colour: background colour of the environment display,
            None setting it to black
        @type background_colour: L{java.awt.Color}
        @return: the simulation GUI instance
        @rtype: L{Simulation2DGUI}
        """
        class CustomGUI(Simulation2DGUI):
            """Custom simulation GUI class to set title and background colour.
            """
            def __init__(self, simulation, name, fields,
                         display_size, background_colour):
                Simulation2DGUI.__init__(self, simulation, name, display_size,
                                         fields)
                self._name = name
                self._background_colour = background_colour
            
            def setupDisplay(self, controller):
                self.createDisplay(controller, self._name,
                                   self._background_colour)
        
        if background_colour == None:
            background_colour = Color.black
        return CustomGUI(self.sim, name, fields, 
                         display_size, background_colour)
    
    def _initSimulation(self):
        """Called to initialise the simulation.
        
        This method creates the agents, and resets them to prepare them
        for the simulation steps.
        """
        agents = create_agents(self._library,
                               agents_init = self._agents_init,
                               world = self)
        for agent in agents:
            agent.reset()
    
    def run(self):
        """Creates the GUI with the previously provided settings.
        
        This method creates the MASON GUI and returns immediately. If the
        GUI is closed, then all threads of the running process are terminated,
        that is, the application will be stopped (see sim.display.Console.doQuit()
        implementation -> calls system.exit(0)). The calling thread should
        not call sys.exit(.), as this will also kill the GUI.
        """
        self.gui.run()
