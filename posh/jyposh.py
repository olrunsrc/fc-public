"""The JyPOSH GUI.

This script creates a simple GUI that allows libraries to be selected,
agents to be run, and customised. The script needs to be run with Jython,
as the GUI uses Java Swing components.

Part of the development of this software was funded by
The Engineering and Physical Sciences Research Council (EPSRC), 
Grant GR/S79299/01 (AIBACS), ``The Impact of Durative Variable state on 
the Design and Control of Action Selection''.
"""

from __future__ import nested_scopes
from POSH.jython_compat import *


# Check if script is invoked by python
if not is_jython:
    print "Error: This script needs to be run with Jython rather than Python!"
    sys.exit(1)
    
import sys, traceback, thread, time
    
from java.lang import Integer
from java.io import File
from java.awt import Dimension, BorderLayout
from java.awt.event import ActionListener, ItemListener, WindowListener
from javax.swing import JFrame, JPanel, JLabel, JComboBox, JCheckBox, JButton, \
    JTextField, JFileChooser, JList, JOptionPane, SwingConstants, BorderFactory, \
    JScrollPane, JTextArea, Timer
from javax.swing.filechooser import FileFilter
from javax.swing.event import ChangeListener, DocumentListener, \
    ListSelectionListener

from POSH import create_agents
from POSH.utils import get_libraries, default_world_script, default_agent_init, \
    get_plans, get_behaviours, run_world_script
from POSH.agentinitparser import parse_agent_init_file, AgentInitParseError, \
    str_to_value
from POSH.logbase import StreamLogger, getRootLogger, \
    WARNING, ERROR, DEBUG, INFO, FATAL

# there is probably a better way to do this, but we need a few libraries which require compiling
# from POSH.utils import compile_mason_java    

# ----------------------------------------------------------------------------
# Log frame
# ----------------------------------------------------------------------------

class LogFrame(JFrame, WindowListener):
    """The log message frame.
    """
    def __init__(self, main_frame):
        """Initialises the log message frame and all its elements.
        """
        JFrame.__init__(self)
        self.setTitle("Log Messages")
        panel = JPanel(BorderLayout())
        self.log_area = JTextArea()
        self.log_area.setEditable(False)
        scroll_pane = JScrollPane()
        scroll_pane.setViewportView(self.log_area)
        panel.add(scroll_pane, BorderLayout.CENTER)
        self.setContentPane(panel)
        self.setSize(400, 300)
        self.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.setVisible(True)
        self.handleWindowClosing(main_frame)
    
    def handleWindowClosing(self, main_frame):
        """Need to handle closing of window as the main frame needs to
        reflect that.
        """
        class WindowCloseListener(WindowListener):
            def __init__(self, main_frame, log_frame):
                self._main_frame = main_frame
                log_frame.addWindowListener(self)
                self.windowOpened = lambda x: None
                self.windowClosed = lambda x: None
                self.windowIconified = lambda x: None
                self.windowDeiconified = lambda x: None
                self.windowActivated = lambda x: None
                self.windowDeactivated = lambda x: None
            def windowClosing(self, event):
                # first need to remove myself to avoid being called from the
                # log_select event handler -> infinite loop
                self._main_frame.log_frame = None
                self._main_frame.log_select.setSelected(False)
                getRootLogger().setLevel(ERROR)
        
        WindowCloseListener(main_frame, self)

# ----------------------------------------------------------------------------
# Agent control frame
# ----------------------------------------------------------------------------

class ControlFrame(JFrame):
    """Frame to control (start/stop/pause) agent(s).
    
    This frame also creates the agent upon construction.
    """
    def __init__(self, agent_init, library, world):
        """Creates the agent and creates the frame that controls it.
        
        agent_init has the form (plan, initialisation), where initialisation
        is a sequence containing (behaviour, attr, value) triples.
        """
        JFrame.__init__(self)
        self.agent_plan, self.agent_init= agent_init
        self.library, self.world = library, world
        self.setTitle("[%s] Control" % self.agent_plan)
        # add components
        self.populate()
        # add agent information
        self.output.append("Agent [%s]:\n" % self.agent_plan)
        if len(self.agent_init) > 0:
            for init_param in self.agent_init:
                self.output.append("%s.%s = %s\n" % (init_param[0], init_param[1],
                                                     str(init_param[2])))
        else:
            self.output.append("No attributes given\n")
        # possible states:
        # 0 - agent not running, but ready to be started
        # 1 - agent running
        # 2 - agent paused
        # 3 - agent starting
        # 4 - agent stopping
        # 5 - failed to create agent, no starting or stopping possible
        # while the agent is running, it is monitored every 100ms by a timer
        # instance that also reacts if the agents is paused/stopped by and
        # outstide instance
        self.gui_state = 0
        self.agent = None
        self.update_state()
        self.pack()
        self.setSize(277, 238)
        self.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.setVisible(True)
    
    def populate(self):
        """Adds components to the panel.
        """
        # clicks on the top button (start/pause/resume)
        class TopButtonListener(ActionListener):
            def __init__(self, frame, start_button):
                self._frame = frame
                start_button.addActionListener(self)
            def actionPerformed(self, event):
                frame = self._frame
                if frame.gui_state == 0:
                    # try to create the agent
                    try:
                        agent = frame.createAgent()
                    except:
                        # failed, bad thing
                        traceback_str = ''.join(
                            traceback.format_exception(sys.exc_type, sys.exc_value, 
                                                       sys.exc_traceback))
                        frame.output.append("\nCreating agent failed:\n%s" % traceback_str)
                        frame.gui_state = 5
                        frame.update_state()
                        return
                    # start the agent
                    if agent.startLoop():
                        frame.output.append("\nStarting Agent")
                        frame.agent = agent
                        frame.gui_state = 3
                        frame.update_state()
                        frame.update_timer.restart()
                    else:
                        frame.output.append("\nResetting Agent failed")
                elif frame.gui_state == 1:
                    # pause the agent
                    if not frame.agent:
                        frame.gui_state = 0
                        frame.update_state()
                        return
                    frame.output.append("\nAgent paused")
                    frame.agent.pauseLoop()
                    frame.gui_state = 2
                    frame.update_state()
                elif frame.gui_state == 2:
                    # resume the agent
                    if not frame.agent:
                        frame.gui_state = 0
                        frame.update_state()
                        return
                    frame.output.append("\nAgent resumed")
                    frame.agent.pauseLoop()
                    frame.gui_state = 1
                    frame.update_state()
        # clicks on the bottom button (stop)
        class BottomButtonListener(ActionListener):
            def __init__(self, frame, stop_button):
                self._frame = frame
                stop_button.addActionListener(self)
            def actionPerformed(self, event):
                frame = self._frame
                # if there's no agent -> reset form
                if not frame.agent:
                    frame.gui_state = 0
                    frame.update_state()
                    return
                # stop agent and update that we are waiting until it's stopped
                if frame.agent.loopStatus()[0]:
                    frame.agent.stopLoop()
                frame.output.append("\nStopping Agent")
                frame.gui_state = 4
                frame.update_state()
                frame.update_timer.restart()
        # listen to event when agent is currently running/starting/stopping
        class UpdateTimerListener(ActionListener):
            def __init__(self, frame):
                self._frame = frame
            def actionPerformed(self, event):
                frame = self._frame
                # if there is no agent, ignore timer (should never happen)
                if not frame.agent:
                    frame.update_timer.stop()
                    return
                loop_status = frame.agent.loopStatus()
                # agent paused (do not update if its starting/stopping)
                if loop_status[0] and loop_status[1] and \
                   (not (frame.gui_state in (2, 3, 4))):
                    frame.output.append("\nAgent Paused")
                    frame.gui_state = 2
                    frame.update_state()
                # agent running (do not update if stopping)
                elif loop_status[0] and not loop_status[1] and \
                    (not (frame.gui_state in (1, 4))):
                    frame.output.append("\nAgent Started")
                    frame.gui_state = 1
                    frame.update_state()
                # agent stopped (do not update if starting)
                if not frame.agent.loopStatus()[0] and \
                    (not (frame.gui_state in (0, 3))):
                    # agent stopped, can be started again
                    frame.update_timer.stop()
                    frame.output.append("\nAgent Stopped")
                    frame.agent = None
                    frame.gui_state = 0
                    frame.update_state()
        # to stop agent if window is closed
        class WindowCloseListener(WindowListener):
            def __init__(self, frame):
                self._frame = frame
                frame.addWindowListener(self)
                self.windowOpened = lambda x: None
                self.windowClosed = lambda x: None
                self.windowIconified = lambda x: None
                self.windowDeiconified = lambda x: None
                self.windowActivated = lambda x: None
                self.windowDeactivated = lambda x: None
            def windowClosing(self, event):
                frame = self._frame
                # is there an agent and is it running?
                if frame.agent and frame.agent.loopStatus()[0]:
                    # stop it
                    frame.agent.stopLoop()
        
        panel = JPanel()
        panel.setLayout(None)
        # add components
        start_button = JButton("Start Agent")
        start_button.setBounds(12, 12, 133, 22)
        panel.add(start_button)
        self.start_button = start_button
        stop_button = JButton("Stop Agent")
        stop_button.setBounds(12, 40, 133, 22)
        panel.add(stop_button)
        self.stop_button = stop_button
        self.output = JTextArea()
        scrolled_output = JScrollPane()
        scrolled_output.setViewportView(self.output)
        scrolled_output.setBounds(12, 74, 247, 124)
        panel.add(scrolled_output)
        status_label = JLabel("Status:")
        status_label.setBounds(151, 16, 104, 15)
        panel.add(status_label)
        self.status_label = status_label
        status = JLabel("Stopped")
        status.setBounds(151, 37, 104, 15)
        panel.add(status)
        self.status = status
        update_timer = Timer(100, UpdateTimerListener(self))
        self.update_timer = update_timer
        self.add(panel)
        TopButtonListener(self, start_button)
        BottomButtonListener(self, stop_button)
        WindowCloseListener(self)
    
    def update_state(self):
        """Update state of buttons and status, based on self.state.
        """
        # start_button, stop_button, state, start_button_text
        new_state = [(True, False, "Not running", "Start Agent"),
                     (True, True, "Running", "Pause Agent"),
                     (True, True, "Paused", "Resume Agent"),
                     (False, False, "Starting", "Start Agent"),
                     (False, False, "Stopping", "Start Agent"),
                     (False, False, "Init failed", "Start Agent")][self.gui_state]
        self.start_button.setEnabled(new_state[0])
        self.start_button.setText(new_state[3])
        self.stop_button.setEnabled(new_state[1])
        self.status.setText(new_state[2])
    
    def createAgent(self):
        """Creates the agent, and sets up the GUI accordingly.
        Additionally gives information about how the agent was created.
        """
        # convert behaviour initialisation
        attr_dict = {}
        for attrs in self.agent_init:
            attr_dict[(attrs[0], attrs[1])] = attrs[2]
        # only a single agent to create
        agents_init = [(self.agent_plan, attr_dict)]
        return create_agents(self.library, 
                             agents_init = agents_init, 
                             world = self.world)[0]
        

# ----------------------------------------------------------------------------
# Main JyPOSH frame
# ----------------------------------------------------------------------------

class Panel(JPanel):
    """Base class providing some utility functions to add components.
    """
    def __init__(self, panel_title):
        """Creates panel with titled border.
        """
        JPanel.__init__(self)
        self.setLayout(None)
        self.setBorder(BorderFactory.createTitledBorder(panel_title))
        # define some methods (n = name, c = caption, b = bounds)
        self._addLabel = lambda n, c, b: self._addComponent(JLabel, n, c, b)
        self._addButton = lambda n, c, b: self._addComponent(JButton, n, c, b)
        self._addComboBox = lambda n, a, b: self._addComponent(JComboBox, n, a, b)
        self._addTextField = lambda n, t, b: self._addComponent(JTextField, n, t, b)
        self._addCheckBox = lambda n, c, b: self._addComponent(JCheckBox, n, c, b)
        self._addList = lambda n, c, b: self._addComponent(JList, n, c, b)
    
    def _addComponent(self, comp_class, name, arg, bounds):
        comp = comp_class(arg)
        self.add(comp)
        comp.setBounds(*bounds)
        setattr(self, name, comp)
    

class LibraryPanel(Panel):
    """The library panel.
    """
    def __init__(self, world_init_panel, agent_init_panel):
        """Initialises the panel by populating it with components.
        """
        Panel.__init__(self, "Behaviour Library")
        self.setBounds(0, 0, 400, 72)
        self.populate(world_init_panel, agent_init_panel)
        # initially disable forms
        world_init_panel.setLibrary(None)
        agent_init_panel.setLibrary(None)
    
    def populate(self, world_init_panel, agent_init_panel):
        """Populates the panel with components and sets their event handlers.
        """
        # class to handle change of library events
        class LibraryChangeListener(ItemListener):
            def __init__(self, world_init_panel, agent_init_panel, cb):
                self.world_init_panel = world_init_panel
                self.agent_init_panel = agent_init_panel
                cb.addItemListener(self)
            def itemStateChanged(self, event):
                library = event.getItem()
                if library == "":
                    library = None
                self.world_init_panel.setLibrary(library)
                self.agent_init_panel.setLibrary(library)
        
        # create components
        self._addLabel("library_label", "Select Behaviour Library",
                       (10, 20, 380, 15))
        self._addComboBox("library_box", [""] + get_libraries(),
                          (10, 41, 380, 22))
        LibraryChangeListener(world_init_panel, agent_init_panel, self.library_box)
        
        
class WorldInitPanel(Panel):
    """The world initialisation panel.
    """
    def __init__(self):
        """Initialises the panel by populating it with components.
        """
        Panel.__init__(self, "World Initialisation")
        self.setBounds(0, 71, 400, 100)
        self.library = None
        self.populate()
            
    def populate(self):
        """Adds components to the panel and assigns event handlers.
        """
        # class to handle 'choose...' button event
        class ChooseButtonListener(ActionListener):
            def __init__(self, script_field, choose_button):
                self.script_field = script_field
                choose_button.addActionListener(self)
            def actionPerformed(self, event):
                # class to filter for directories and python scripts
                class python_filter(FileFilter):
                    def accept(self, f):
                        return f.isDirectory() or \
                               f.getName()[-3:].lower() == '.py'
                    def getDescription(self):
                        return "Python scripts"
                # show file chooser dialog
                chooser = JFileChooser()
                chooser.addChoosableFileFilter(python_filter())
                chooser.setCurrentDirectory(File(self.script_field.getText()))
                if chooser.showOpenDialog(self.script_field) == chooser.APPROVE_OPTION:
                    self.script_field.setText(
                        chooser.getSelectedFile().getAbsolutePath())
        
        # create components
        self._addLabel("world_label", "World Initialisation Script",
                       (10, 20, 280, 15))
        self._addTextField("script_field", "", (10, 41, 275, 22))
        self._addButton("choose_button", "Choose...", (291, 41, 99, 22))
        ChooseButtonListener(self.script_field, self.choose_button)
    
    def resetState(self, enabled = True):
        """Resets the components to their default state.
        
        If enabled is set to False, then the components are disabled.
        """
        if self.library == None:
            self.script_field.setText("")
        else:
            self.script_field.setText(default_world_script(self.library))
        # default_script needs to go first, as it modifies the 'enabled' state
        # of other components
        for comp in (self.world_label, self.script_field,
                     self.choose_button, self):
            comp.setEnabled(enabled)
    
    def setLibrary(self, library):
        """Resets the panel to use the given library.
        
        If None is given, then the panel is diabled.
        """
        self.library = library
        if library == None:
            self.resetState(False)
        else:
            self.resetState()
   
    def getWorldScript(self):
        """Returns the current world script.
        """
        return self.script_field.getText()
    

class AgentInitPanel(Panel):
    """The agent initialisation panel.
    """
    def __init__(self, frame):
        """Initialises the panel by populating it with components.
        """
        Panel.__init__(self, "Agent Initialisation")
        self.setBounds(0, 171, 400, 391)
        self.frame = frame
        self.library = None
        self.agents = []
        # create the separate parts
        self.addAgentInitFileSection()
        self.addAgentsSection()
        self.addAttributesSection()
        self.addClearSection()
        
    def addAgentInitFileSection(self):
        """Creates the buttons at the top of the panel.
        """
        class ChooseButtonListener(ActionListener):
            def __init__(self, script_field, choose_button):
                self.script_field = script_field
                choose_button.addActionListener(self)
            def actionPerformed(self, event):
                # class to filter for directories and init files
                class python_filter(FileFilter):
                    def accept(self, f):
                        return f.isDirectory() or \
                               f.getName()[:5].lower() == 'init_'
                    def getDescription(self):
                        return "init_ file"
                # show file chooser dialog
                chooser = JFileChooser()
                chooser.addChoosableFileFilter(python_filter())
                chooser.setCurrentDirectory(File(self.script_field.getText()))
                if chooser.showOpenDialog(self.script_field) == chooser.APPROVE_OPTION:
                    self.script_field.setText(
                        chooser.getSelectedFile().getAbsolutePath())
        # class to handle 'load default init' button event
        class LoadButtonListener(ActionListener):
            def __init__(self, agent_init_panel, init_script):
                self.agent_init_panel = agent_init_panel
                init_script.addActionListener(self)
            def actionPerformed(self, event):
                panel = self.agent_init_panel
                
                if panel.script_field.getText() != None and panel.script_field.getText() != "":
                    try:
                        agent_init = parse_agent_init_file(panel.script_field.getText())
                    except AgentInitParseError, e:
                        JOptionPane.showMessageDialog(self.agent_init_panel,
                            "Error when parsing agent initialisation file:\n%s" % str(e),
                            "Error", JOptionPane.ERROR_MESSAGE)
                        return
                    # convert to own format (dictionary is not helpful in the GUI case)
                    agents = []
                    for agent in agent_init:
                        attr_list = []
                        for k, v in agent[1].items():
                            attr_list.append(list(k) + [v])
                        agents.append((agent[0], attr_list))
                    panel.agents = agents
                    panel.rebuildAgentList()
                    
                else:
                    pass
                    
        # create components
        self._addLabel("init_file_label", "Agent Initialisation File",
                       (10, 20, 280, 15))
        self._addTextField("script_field", "", (10, 41, 275, 22))
        self._addButton("choose_button", "Choose...", (291, 41, 99, 22))
        self._addButton("init_script", "Load Agent Init File",
                        (10, 69, 380, 22))
        ChooseButtonListener(self.script_field, self.choose_button)
        LoadButtonListener(self, self.init_script)
    
    def addAgentsSection(self):
        """creates the agents section of the panel.
        """
        # class to handle changed agents
        class AgentBoxItemListener(ItemListener):
            def __init__(self, agent_init_panel, agents_box):
                self.agent_init_panel = agent_init_panel
                agents_box.addItemListener(self)
            def itemStateChanged(self, event):
                self.agent_init_panel.rebuildAttributeList()
        # class to handle "Add..." button clicks
        class AddButtonListener(ActionListener):
            def __init__(self, agent_init_panel, agents_box, agent_add):
                self.agent_init_panel = agent_init_panel
                self.agents_box = agents_box
                agent_add.addActionListener(self)
            def actionPerformed(self, event):
                plans = get_plans(self.agent_init_panel.library)
                if len(plans) < 1:
                    JOptionPane.showMessageDialog(self.agent_init_panel,
                        "No plans found for given behaviour library!",
                        "Error", JOptionPane.ERROR_MESSAGE)
                    return
                choice = JOptionPane.showInputDialog(
                    self.agent_init_panel, "Please choose agent's plan:",
                    "Choose Plan", JOptionPane.PLAIN_MESSAGE, None,
                    plans, plans[0])
                if choice != None and len(choice) > 0:
                    self.agent_init_panel.agents.append((choice, []))
                    self.agent_init_panel.rebuildAgentList()
                    self.agents_box.setSelectedIndex(
                        self.agents_box.getItemCount() - 1)
        # class to handle "Remove" button clicks
        class RemoveButtonListener(ActionListener):
            def __init__(self, agent_init_panel, agents_box, agent_remove):
                self.agent_init_panel = agent_init_panel
                self.agents_box = agents_box
                agent_remove.addActionListener(self)
            def actionPerformed(self, event):
                agent_idx = self.agents_box.getSelectedIndex()
                if agent_idx > -1:
                    del self.agent_init_panel.agents[agent_idx]
                    self.agent_init_panel.rebuildAgentList()

        # create components
        self._addLabel("agents_label", "Agents", (10, 104, 380, 15))
        self._addComboBox("agents_box", [], (10, 122, 208, 22))
        self._addButton("agent_add", "Add...", (224, 122, 75, 22))
        self._addButton("agent_remove", "Remove", (305, 122, 85, 22))
        AddButtonListener(self, self.agents_box, self.agent_add)
        RemoveButtonListener(self, self.agents_box, self.agent_remove)
        AgentBoxItemListener(self, self.agents_box)
    
    def addAttributesSection(self):
        """create the Attributes section of the panel.
        """
        # class to handle changes of text field
        class AttributeChangeListener(DocumentListener):
            def __init__(self, attr_field, value_field, attr_add):
                self.attr_field = attr_field
                self.value_field = value_field
                self.attr_add = attr_add
                attr_field.getDocument().addDocumentListener(self)
                value_field.getDocument().addDocumentListener(self)
            def changedUpdate(self, event):
                if len(self.attr_field.getText().strip()) > 0 and \
                   len(self.value_field.getText().strip()) > 0:
                    self.attr_add.setEnabled(True)
                else:
                    self.attr_add.setEnabled(False)
            def insertUpdate(self, event):
                self.changedUpdate(event)
            def removeUpdate(self, event):
                self.changedUpdate(event)
        # class to handle adding attributes to agents
        class AttributeAddListener(ActionListener):
            def __init__(self, agent_init_panel, attr_add):
                self.agent_init_panel = agent_init_panel
                attr_add.addActionListener(self)
            def actionPerformed(self, event):
                panel = self.agent_init_panel
                agent_idx = panel.agents_box.getSelectedIndex()
                panel.agents[agent_idx][1].append(
                    (panel.beh_box.getSelectedItem(),
                     panel.attr_field.getText(),
                     str_to_value(panel.value_field.getText())))
                panel.attr_field.setText("")
                panel.value_field.setText("")
                panel.rebuildAttributeList()
        # class to handle selecting element in the item list
        class AttributeListSelectionListener(ListSelectionListener):
            def __init__(self, attr_remove, attrs_list):
                self.attr_remove = attr_remove
                self.attrs_list = attrs_list
                attrs_list.getSelectionModel().addListSelectionListener(self)
            def valueChanged(self, event):
                attr_idx = self.attrs_list.getSelectedIndex()
                self.attr_remove.setEnabled(attr_idx > 0)
        # class to handle the 'Remove' button for attributes
        class AttributeRemoveListener(ActionListener):
            def __init__(self, agent_init_panel, attr_remove):
                self.agent_init_panel = agent_init_panel
                attr_remove.addActionListener(self)
            def actionPerformed(self, event):
                panel = self.agent_init_panel
                agent_idx = panel.agents_box.getSelectedIndex()
                attr_idx = panel.attrs_list.getSelectedIndex() - 1
                del panel.agents[agent_idx][1][attr_idx]
                panel.rebuildAttributeList()
        # class to handle 'clear all settings' button event

        # create components
        self._addLabel("attrs_label", "Attributes", (10, 153, 380, 15))
        self._addLabel("beh_label", "Behaviour", (17, 170, 66, 15))
        self._addLabel("attr_label", "Attribute", (17, 202, 66, 15))
        self._addLabel("value_label", "Value", (17, 230, 66, 15))
        self.beh_label.setHorizontalAlignment(SwingConstants.TRAILING)
        self.attr_label.setHorizontalAlignment(SwingConstants.TRAILING)
        self.value_label.setHorizontalAlignment(SwingConstants.TRAILING)
        self._addComboBox("beh_box", [], (89, 170, 210, 22))
        self._addTextField("attr_field", "", (89, 198, 210, 22))
        self._addTextField("value_field", "", (89, 226, 210, 22))
        self._addButton("attr_add", "Add", (305, 170, 85, 22))
        # list within a scrollpane
        self.attrs_list = JList([])
        self.scroll_view = JScrollPane(self.attrs_list)
        self.scroll_view.setBounds(10, 255, 289, 97)
        self.add(self.scroll_view)
        self._addButton("attr_remove", "Remove", (305, 255, 85, 22))
        AttributeChangeListener(self.attr_field, self.value_field, self.attr_add)
        AttributeAddListener(self, self.attr_add)
        AttributeListSelectionListener(self.attr_remove, self.attrs_list)
        AttributeRemoveListener(self, self.attr_remove)
    
    def addClearSection(self):
        """create the Clear Agent section of the panel.
        """
        class ClearAllButtonListener(ActionListener):
            def __init__(self, agent_init_panel, clear_all):
                self.agent_init_panel = agent_init_panel
                clear_all.addActionListener(self)
            def actionPerformed(self, event):
                self.agent_init_panel.clearSettings()
        
        self._addButton("clear_all", "Clear All Settings", (10, 359, 380, 22))
        ClearAllButtonListener(self, self.clear_all)
    
    def resetState(self, enabled = True):
        """Resets the components to their default state.
        
        If enabled is set to False, then the components are disabled.
        """
#        # debugging code 
#        JOptionPane.showMessageDialog(None,
#                            "And Worldinit says %s %s %s" % 
#                                (self.library, default_agent_init(self.library), time.gmtime()),
#                            "Error", JOptionPane.ERROR_MESSAGE)
        if self.library == None:
            self.script_field.setText("")
        else:
            self.script_field.setText(default_agent_init(self.library))

        for comp in (self.script_field, self.choose_button,
                     self.init_script, self.clear_all,
                     self.agents_label, self.agents_box,
                     self.agent_add, self.attrs_label, self.beh_label,
                     self.attr_label, self.value_label, self.beh_box,
                     self.attr_field, self.value_field, self.attr_add,
                     self.attrs_list, self.attr_remove,
                     self):
            comp.setEnabled(enabled)
        self.clearSettings()
    
    def clearSettings(self):
        """Remove all settings (except for the library and its default script_field).
        
        This method might also disable some controls, but does not enable any.
        """
        self.agents = []
        self.rebuildAgentList()
        self.beh_box.removeAllItems()
        if self.library != None:
            for beh_class in get_behaviours(self.library):
                self.beh_box.addItem(beh_class.__name__)
        self.attr_field.setText("")
        self.value_field.setText("")
        self.attr_add.setEnabled(False)
        self.rebuildAttributeList()
        self.attr_remove.setEnabled(False)
        
    def setLibrary(self, library):
        """Resets the panel to use the given library.
        
        If None is given, then the panel is diabled.
        """
        self.library = library
        self.resetState(library != None)
    
    def rebuildAgentList(self):
        """Rebuilds the list of agents in the combobox.
        """
        agents_box = self.agents_box
        agents_box.removeAllItems()
        agent_idx = 1
        for agent in self.agents:
            agents_box.addItem("%d: %s" % (agent_idx, agent[0]))
            agent_idx += 1
        # no items -> disable remove
        if agent_idx == 1:
            enable = False
            self.agent_remove.setEnabled(False)
            self.frame.run_button.setEnabled(False)
        else:
            enable = True
            self.agents_box.setSelectedIndex(0)
        for comp in (self.agent_remove, self.beh_label,
                     self.attr_label, self.value_label, self.beh_box,
                     self.attr_field, self.value_field, self.attr_add,
                     self.attrs_list, self.frame.run_button):
            comp.setEnabled(enable)
            if len(self.attr_field.getText().strip()) == 0 or \
               len(self.value_field.getText().strip()) == 0:
                self.attr_add.setEnabled(False)
        self.rebuildAttributeList()
    
    def rebuildAttributeList(self):
        """Rebuilds the list of attributes in the list box.
        """
        if len(self.agents) == 0:
            string_list = []
        else:
            agent_idx = self.agents_box.getSelectedIndex()
            string_list = ["[%s]" % self.agents[agent_idx][0]] + \
                ["%s.%s = %s" % (element[0], element[1], str(element[2])) \
                 for element in self.agents[agent_idx][1]]
        # create list model through ugly hack
        list_model = JList(string_list).getModel()
        self.attrs_list.setModel(list_model)
        

class JyPOSHGUI(JFrame):
    """The main frame.
    """
    def __init__(self):
        """Initialises the main frame and all its elements.
        """
        JFrame.__init__(self)
        # default values
        self.library = None
        # add GUI elements
        self.populate()
        self.setupLogHandling()
    
    def populate(self):
        """Populates the GUI with elements.
        """
        # calls runSim() when run button is pressed
        class RunButtonListener(ActionListener):
            def __init__(self, frame, run_button):
                self.frame = frame
                run_button.addActionListener(self)
            def actionPerformed(self, event):
                self.frame.runSim()
        # handles changing of the log level setting
        class LogLevelChangeListener(ItemListener):
            def __init__(self, frame, log_level):
                self._frame = frame
                log_level.addItemListener(self)
            def itemStateChanged(self, event):
                # only change if log window is shown
                if self._frame.log_frame:
                    new_level = {"Fatal" : FATAL, "Error" : ERROR,
                                 "Warning" : WARNING, "Info" : INFO,
                                 "Debug" : DEBUG}[event.getItem()]
                    getRootLogger().setLevel(new_level)
        # handles showing/hiding the log level window
        class LogFrameActionListener(ActionListener):
            def __init__(self, frame, log_select):
                self._frame = frame
                log_select.addActionListener(self)
            def actionPerformed(self, event):
                if event.getSource().isSelected():
                    if self._frame.log_frame:
                        return
                    self._frame.log_frame = LogFrame(self._frame)
                    # set current level (reset when window is hidden)
                    level = [FATAL, ERROR, WARNING, INFO, DEBUG][self._frame.log_level.getSelectedIndex()]
                    getRootLogger().setLevel(level)
                else:
                    if not self._frame.log_frame:
                        return
                    # set to lowest level to avoid handling to many messages
                    getRootLogger().setLevel(ERROR)
                    self._frame.log_frame.dispose()
                    self._frame.log_frame = None
                    
        #make sure you have all the files you need -- there must be a better way to do this!  see launch.py & POSH/utils
 #       compile_mason_java() 

        # need to create button first, as its state is controlled by the
        # agent_init_panel (cannot start sim without agent - doh)
        run_button = JButton("Initialise World and Agent(s)")
        run_button.setBounds(10, 567, 380, 22)
        self.run_button = run_button
        self.log_frame = None
        self.world_init_panel = WorldInitPanel()
        self.agent_init_panel = AgentInitPanel(self)
        self.library_panel = LibraryPanel(self.world_init_panel, self.agent_init_panel)
        panel = JPanel()
        panel.setLayout(None)
        panel.add(self.library_panel)
        panel.add(self.world_init_panel)
        panel.add(self.agent_init_panel)
        panel.add(run_button)
        RunButtonListener(self, run_button)
        # add log message controls
        log_select = JCheckBox("Show Log Message Window")
        log_select.setBounds(10, 595, 200, 19)
        panel.add(log_select)
        LogFrameActionListener(self, log_select)
        self.log_select = log_select
        log_level = JComboBox(["Fatal", "Error", "Warning", "Info", "Debug"])
        log_level.setSelectedIndex(2)
        log_level.setBounds(290, 595, 99, 22)
        panel.add(log_level)
        LogLevelChangeListener(self, log_level)
        self.log_level = log_level
        self.setContentPane(panel)
        self.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE)
        self.setTitle("JyPOSH GUI")
        self.pack()
        self.setSize(410, 662)
    
    def setupLogHandling(self):
        """Sets up the log message handler.
        """
        class MessageHandler(StreamLogger):
            def __init__(self, frame):
                StreamLogger.__init__(self)
                self._frame = frame
            def write(self, msg):
                if self._frame.log_frame:
                    self._frame.log_frame.log_area.append(msg)
        self.logger = MessageHandler(self)
        
    def runSim(self):
        """First runs the world initialisation script and then creates the
        agent frames to control the agents (given that the world
        initialisation script didn't create and run them).
        """
        library = self.library_panel.library_box.getSelectedItem()
        world_script = self.world_init_panel.getWorldScript()
        # convert agent initialisation structure
        agents = self.agent_init_panel.agents
        agents_init = []
        for agent in agents:
            attr_dict = {}
            for attrs in agent[1]:
                attr_dict[(attrs[0], attrs[1])] = attrs[2]
            agents_init.append((agent[0], attr_dict))
        # run world initialisation script
        # try to read the file first, to catch IO exception (otherwise it might
        # have been caused by the script itself)
        world = None
        if world_script != "":
            try:
                f = open(world_script, 'r')
            except IOError:
                JOptionPane.showMessageDialog(self.agent_init_panel,
                    "Error opening the world initialisation script\n%s" % world_script,
                    "Error", JOptionPane.ERROR_MESSAGE)
                return
            # run the script itself
            self._world_init_result = None
            thread.start_new_thread(self._world_init_thread,
                                    (world_script, library, "", agents_init))
            while self._world_init_result == None:
                time.sleep(0.1)
            if self._world_init_result[2] != None:
                JOptionPane.showMessageDialog(self.agent_init_panel,
                    "An error occurred when running the world initialisation " \
                    "script:\n%s" % self._world_init_result[2],
                    "Error", JOptionPane.ERROR_MESSAGE)
                return
            elif self._world_init_result[1]:
                # don't need to run agents if that was handled by world
                # initialisation script
                return
            world = self._world_init_result[0]
        # create control frames, one per agent
        for agent in agents:
            ControlFrame(agent, library, world)
    
    def _world_init_thread(self, world_script, library, world_args, agents_init):
        """Runs the world initialisation script.
        
        When the script is done or completes with an exception, then
        self._world_init_result is set as a triple
        (world_obj, ran_agents, exception), where the last element is either
        set to None if all went well, or contains the string describing the
        execption that occured while running the script.
        """
        traceback_str, world_obj, ran_agents = None, None, None
        try:
            world_obj, ran_agents = \
                run_world_script(world_script, library, "", agents_init)
        except:
            traceback_str = ''.join(
                traceback.format_exception(sys.exc_type, sys.exc_value, 
                                               sys.exc_traceback))
        self._world_init_result = (world_obj, ran_agents, traceback_str)

def compile_mason_java():
    import os
    ext1='.java'
    ext2='.class'
  #  dir=os.getcwd()+'/platform_files/MASON/'
    dir=sys.path[0]+'/platform_files/MASON/'
    # if not os.path.exists (dir) or not os.access(dir, os.R_OK and os.W_OK and os.X_OK): # acess doesn't work in jython
    try:
        java_src=filter((lambda str:str!="__init__"),map((lambda str:str[:len(str)-len(ext1)]),filter((lambda str:str.endswith(ext1)),os.listdir(dir)))) 
        classes=filter((lambda str:str!="__init__$py"),map((lambda str:str[:len(str)-len(ext2)]),filter((lambda str:str.endswith(ext2)),os.listdir(dir)))) 
        
        if java_src!=classes:
            cmd = 'javac -cp %smason.jar %s*.java' % (dir,dir)
            os.system(cmd)
    except:
        print "Cannot access file system (tried "+dir+")  If you are using MASON, please ensure MASON jar files are compiled (see README)." 
        print "Error:", sys.exc_info()[0]

if __name__ == '__main__':
    compile_mason_java()  #move to after mason library selected?
    gui = JyPOSHGUI()
    gui.setVisible(True)