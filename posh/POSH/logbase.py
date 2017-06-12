"""Module providing the logging base class and some utilities.

Depending on if this module is loaded in Python or Jython, is uses the
Python 'logging' module or the Jython 'log4j' module.

If the log4j module is used, it is let uninitialised and needs to be
initialised by the simulation that uses POSH.
"""

# Jython compatibility
from __future__ import nested_scopes
from jython_compat import *

# Logging modules, differentiate between python and jython
# also set log levels
if is_jython:
    from java.io import OutputStream
    from org.apache.log4j import Logger, Level, BasicConfigurator, \
        WriterAppender, PatternLayout
    getLogger = Logger.getLogger
    getRootLogger = Logger.getRootLogger
    ALL = Level.ALL
    FATAL = Level.FATAL
    ERROR = Level.ERROR
    INFO = Level.INFO
    WARNING = Level.WARN
    DEBUG = Level.DEBUG
    def setup_console_logging(level = INFO):
        """Sets up basic console logging at the given log level.
        """
        BasicConfigurator.configure()
        getRootLogger().setLevel(level)
    
    class StreamLogger:
        """Class to direct streamed log messages to some output.
        
        To use this class, it needs to be inherited and its write() method needs
        to be overridden to handle new incoming log messages.
        """
        def __init__(self):
            """Initialises the stream logger.
            """
            class StreamHandler(OutputStream):
                def __init__(self, handler):
                    self._handler = handler
                def write(self, b, offset = None, length = None):
                    if offset != None and length != None:
                        self._handler.write(''.join(map(chr, b[offset:offset + length])))
                    else:
                        self._handler.write(''.join(map(chr, b)))
            writer = StreamHandler(self)
            layout = PatternLayout(PatternLayout.TTCC_CONVERSION_PATTERN)
            writer = StreamHandler(self)
            appender = WriterAppender(layout, writer)
            getRootLogger().addAppender(appender)
            
        def write(self, msg):
            """Handes the new debug message.
            
            This method needs to be overridden by an inheriting class to handle
            log messages. By default, it print the message to the console.
        
            @param msg: The log message. If a newline is required, then the
                message contains the required '\n' character.
            @type msg: string
            """
            print msg,
# else pytyhon...         
else:
    import logging
    getLogger = logging.getLogger
    getRootLogger = lambda: getLogger()
    getLogger.__doc__ = "Returns a logger with the given name. The name is " \
        "typically a dot-separated hierarchical name like 'a', 'a.b' or " \
        "'a.b.c.d'."
    getRootLogger.__doc__ = "Returns the logger at the root of the " \
        "logging hierarchy."
    ALL = logging.NOTSET
    FATAL = logging.CRITICAL
    ERROR = logging.ERROR
    INFO = logging.INFO
    WARNING = logging.WARNING
    DEBUG = logging.DEBUG
    def setup_console_logging(level = INFO):
        """Sets up basic console logging at the given log level.
        """
        logging.basicConfig(level = level)

    class StreamLogger(logging.StreamHandler):
        """Class to direct streamed log messages to some output.
    
        To use this class, it needs to be inherited and its write() method needs
        to be overridden to handle new incoming log messages.
        """
        def __init__(self, level = INFO):
            """Initialises the stream logger.
            """
            class StreamHandler:
                def __init__(self, handler):
                    self._handler = handler
                def write(self, msg):
                    self._handler.write(msg)
                def flush(self):
                    pass
            # redirect log messages to class instance's write() method
            logging.StreamHandler.__init__(self, StreamHandler(self))
            self.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
            logging.getLogger().addHandler(self)
    
        def write(self, msg):
            """Handes the new debug message.
            
            This method needs to be overridden by an inheriting class to handle
            log messages. By default, it print the message to the console.
        
            @param msg: The log message. If a newline is required, then the
                message contains the required '\n' character.
            @type msg: string
            """
            print msg,

# set root logger to process all messages
getRootLogger().setLevel(ALL)


class LogBase:
    """Base for agent-based log messages.
    
    This class adds the object variable 'log' to each instance of a subclass
    that inherits this class. This log variable is a logging object that is
    to be used for creating log messages.
    """
    def __init__(self, agent, log_name, default_level = None):
        """Initialises the logger.
        
        The logger is initialised to send log messages
        under the logging domain [AgentId].[log_name]. The 
        name of the agent is retrieved by accessing C{agent.id}
        variable.
        
        If the logger is initialised for the agent itself,
        logName has to be set to an empty string.
        
        @param agent: A POSH agent.
        @type agent: L{POSH.AgentBase}
        @param log_name: Name of the logging domain, "" if called
            for the agent.
        @type log_name: string
        @param default_level: The default logging level.
        @type default_level: int
        """
        # workaround for scheduled POSH, where not all plan elements are
        # initialised with an agent -> the given 'agent' attribute does
        # not have an id
        agent_id = getattr(agent, 'id', 'NOID')
        if log_name == "":
            # called for the agent
            log_domain = agent_id
        else:
            log_domain = "%s.%s" % (agent_id, log_name)
        self.log = getLogger(log_domain)
        if default_level:
            self.log.setLevel(default_level)
