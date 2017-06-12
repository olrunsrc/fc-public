#  BODbot created as a means of evaluating Behaviour Oriented Design [BOD]
#  Much code here re-used from Andy Kwong's poshbot
#  It has been refactored on the 29/08/07 to make Bot a behaviour and clean
#  up the behaviour structure a bit.

# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

#  We need to start a comms thread in order to get updates
#  to the agent status from the server.
from socket import *
from POSH import Behaviour
from POSH.utils import current_time
import re #re is for Regular Expressions
import thread
import sys
import time

import utilityfns

# import behaviour classes
import movement
import combat

# Previously used to initialise the agent. The changes now are:
# - Bot is a behaviour of the agent, can be accessed through self.agent.Bot
# - The connection to UT is established on calling reset() of the agent
#   (done from startLoop()) that calls reset() of all behaviours -> In the
#   Bot behaviour that causes the agent to connect.
# - The state objects PosInfo and CombatInfo are not referenced from many
#   behaviours but only one behaviour. At some point they should be merged
#   with the behaviour object itself (more coherent, behaviours can have states).

#def make_behavior(ip, port, botname, agent, *args, **kw):
    #bot = Bot_Agent(agent, ip, port, botname) # Bot_Agent keeps a local copy of the bot state
    #BList = []
    
    #agent.bot = bot
    
    ## Andy's primitives
    #ab = andybehaviour.AndyBehavior(agent = agent)
    #ab.bind_bot(bot) #sets ab's bot to the arg sent
    ##ab.bot.connect() now done below
    #BList.append(ab)
    
    #PosInfo = movement.PositionsInfo()
    #CombatInfo = combat.CombatInfoClass()
    
    #mb = movement.MovementBehaviour(PosInfo, CombatInfo, agent = agent)
    #mb.bind_bot(bot)
    #BList.append(mb)
    
    #sb = status.StatusBehaviour(PosInfo, agent = agent)
    #sb.bind_bot(bot)
    #BList.append(sb)
    
    #cb = combat.CombatBehaviour(PosInfo, CombatInfo, agent = agent)
    #cb.bind_bot(bot)
    #BList.append(cb)
    
    #bot.connect()

    #return BList


class Bot(Behaviour):
    """The Bot behaviour.
    
    This behaviour does not provide any actions that are directly used in plans.
    Rather, it establishes the connection with UT and provides methods to
    control the bot which can be used by other behaviours.
    
    The behaviour keeps a local copy of the bot state. Gamebots do not support
    queries on the agent sense, it sends a copy of the environment to the
    agent periodically.
    
    To change connection IP, port and the bot's name, use the attributes
    Bot.ip, Bot.port and Bot.botname.
    """
    def __init__(self, agent, attributes = None):
        Behaviour.__init__(self, agent, (), (), attributes)
        # default connection values, use attributes to override
        self.ip = "127.0.0.1"
        self.port = "3000"
        self.botname = "BODbot"
        
        # Valid values for team are 0 and 1. If an invalid value is used gamebots
        # will alternate the team on which each new bot is placed.
        self.team = "-1"
        
        # all the rest is standard
        self.events = [] #things like hitting a wall
        self.conninfo = {}
        self.gameinfo = {}
        self.view_players = {}
        self.view_items = {}
        self.nav_points = {}
        self.botinfo = {}
        self.s_gameinfo = {}
        self.s_view_players = {}
        self.s_view_items = {}
        self.s_nav_points = {}
        self.s_botinfo = {}
        self.msg_log = [] # Temp Log for message received
        self.msg_log_max = 4096 # Max Temp Log size
        self.sent_msg_log = [] # Temp Log for messages sent
        self.sent_msg_log_max = 6 # Max Temp Log size
        self.hit_timestamp = 0 # Used to inhibit was_hit()
        self.thread_active = 0
        self.kill_connection = 0
        self.rotation_hist = []
        self.velocity_hist = []
        self.thread_active = 0
        self.conn_ready = 0
        self.conn_thread_id = None
    
    def reset(self):
        """Attempts connecting to the UT server.
        
        If the bot is currently connected, it disconnects, waits a bit,
        and then reconnects.
        """
        # disconnect first
        if self.thread_active:
            self.log.debug("Currently connected, trying to disconnect")
            # wait for 3 seconds for disconnection
            timeout = 0
            while self.thread_active and timeout < 30:
                timeout += 1
                time.sleep(0.1)
        # connect only if not connected
        if not self.thread_active:
            self.connect()
            return True
        else:
            self.log.error("Reset failed, as failed to disconnect")
            return False

    def checkError(self):
        """Returns if the behaviour is ready.
        
        This method is called by agent._loop_thread to make sure that the
        behaviour is OK in every cycle. This behaviour is OK if it is connected
        to UT. Otherwise it isn't.
        """
        if self.conn_ready:
            return False
        else:
            return True

    def exitPrepare(self):
        """Prepares the bot to exit by disconnecting from UT.
        """
        self.disconnect()

    def proc_item(self, string):
        (cmd, varstring) = re.compile('\s+').split(string, 1) #\s is a special escape character, which matches any white-space, varstring will hold flags returned from regular expression creation (see sre.py)
        vars = re.compile('\{(.*?)\}').findall(varstring)
        var_dict = {}
        for var in vars:
            (attr, value) = re.compile('\s+').split(var, 1)
            var_dict[attr] = value
        if cmd == "DAM" or cmd == "PRJ":
            var_dict["timestamp"] = current_time()
        return (cmd, var_dict)

    # Calls connect_thread in a new thread
    def connect(self):
        self.log.info("Connecting to Server (%s:%s)" % (str(self.ip), str(self.port)))
        if not self.conn_thread_id:
            self.thread_active = 1
            self.conn_thread_id = thread.start_new_thread(self.connect_thread, ())
            return 1
        else:
            self.log.error("Attempting to Connect() when thread already active")
            return 0

    # This method runs inside a thread updating the agent state
    # by reading from the network socket
    def connect_thread(self):
        self.kill_connection = 0
        try:
            self.sockobj = socket(AF_INET, SOCK_STREAM)
            self.sockobj.connect((self.ip, int(self.port)))
            self.sockin = self.sockobj.makefile('r')
            self.sockout = self.sockobj.makefile('w')
        except:
            self.log.error("Connection to server failed")
            self.kill_connection = 1 # Skip the read loops
        else:
            self.log.error("Connected to server")
            self.kill_connection = 0
        
        # This loop waits for the first NFO message
        while not self.kill_connection:
            try:
                x = self.sockin.readline()
            except:
                self.log.error("Connection Error on readline()")
                self.kill_connection = 1
                break
                
            if not x:
                self.log.error("Connection Closed from Remote End")
                self.kill_connection = 1
                break
            
            #print x
            (cmd, dict) = self.proc_item(x)
            if cmd == "NFO":
                # Send INIT message
                self.conninfo = dict
                self.send_message("INIT", {"Name" : self.botname, "Team" : self.team})
                self.conn_ready = 1 # Ready to send messages
                break

        # Now the main loop
        # Not everything is implemented. Just some basics
        while not self.kill_connection:
            try:
                x = self.sockin.readline()
            except:
                self.log.error("Connection Error on readline()")
                break
                
            if not x:
                self.log.error("Connection Closed from Remote End")
                break
            
            #print "R>> " +  str(self) + x
            (cmd, dict) = self.proc_item(x)
            sync_states = ("SLF","GAM","PLR","NAV","MOV","DOM","FLG","INV")
            events = ("WAL", "BMP")
            self.msg_log.append((cmd, dict))
            if cmd == "BEG":
                # When a sync batch is arriving, make sure the shadow
                # states are cleared
                self.s_gameinfo = {}
                self.s_view_players = {}
                self.s_view_items = {}
                self.s_nav_points = {}
                self.s_botinfo = {}
            elif cmd in sync_states:
                # These are sync. messages, handle them with another method
                self.proc_sync(cmd, dict)
            elif cmd == "END":
                # When a sync batch ends, we want to make the shadow
                # states that we were writing to to be the real one
                self.gameinfo = self.s_gameinfo
                self.view_players = self.s_view_players
                self.view_items = self.s_view_items
                self.nav_points = self.s_nav_points
                self.botinfo = self.s_botinfo
                # Also a good time to trim the events list
                # Only keep the last 50 events
                self.events = self.events[-50:]
                self.msg_log = self.msg_log[-1000:]
            elif cmd in events:
                # The bot hit a wall or an actor, make a note
                # of it in the events list with timestamp
                self.events.append((current_time(), cmd, dict))
            elif cmd == "SEE":
                # Update the player positions
                self.view_players[dict["Id"]] = dict
            elif cmd == "PTH":
                # pass the details to the movement behaviour
                self.agent.Movement.receive_pth_details(dict)
            elif cmd == "RCH":
                self.agent.Movement.receive_rch_details(dict)
            elif cmd == "PRJ": # incoming projectile
                self.agent.Combat.receive_prj_details(dict)
            elif cmd == "DAM": # damage taken
                self.agent.Combat.receive_dam_details(dict)
            elif cmd == "KIL": # some other player died
                self.agent.Combat.receive_kil_details(dict)
            elif cmd == "DIE": # this player died
                self.agent.Combat.receive_die_details(dict)
            else:
                pass

        self.log.info("Closing Sockets and Cleaning Up...")
        try:
            self.sockout.flush()
            self.sockout.close()
            self.sockin.close()
            self.sockobj.close()
        except:
            self.log.error("Error closing files and sockets")
            
        self.thread_active = 0
        self.conn_ready = 0
        self.conn_thread_id = None
        self.log.info("Connection Thread Terminating...")

    def disconnect(self):
        self.kill_connection = 1

    def send_message(self, cmd, dict):
        string = cmd
         
        self.sent_msg_log.append((cmd, dict))
        # does the list need truncating?
        if len(self.sent_msg_log) > self.sent_msg_log_max:
            del self.sent_msg_log[0 : -self.sent_msg_log_max]
        
        for (attr, value) in dict.items():
            # Only works when using str() otherwise error because target is a tuple
            # not all targets are tuples, find out why. FA
            string = string + " {" + attr + " " + str(value) + "}"
        #print "About to send " + string
        string = string + "\r\n"
        # print >> self.sockout, string
        # print "S>> " + string
        try:
            self.sockout.write(string)
            self.sockout.flush()
        except:
            self.log.error("Message : %s unable to send" % string)
            return 0
        else:
            return 1

    #handles synchronisation messages
    def proc_sync(self, command, values):
        if command == "SLF": #info about bot's state
            self.s_botinfo = values
            # Keep track of orientation so we can tell when we are moving
            # Yeah, we only need to know the Yaw
            self.rotation_hist.append(int(
                re.search(',(.*?),', values['Rotation']).group(1)))
            # Trim list to 3 entries
            if len(self.rotation_hist) > 3:
                del(self.rotation_hist[0])
            # Keep track of velocity so we know when we are stuck
            self.velocity_hist.append(self.calculate_velocity( \
                values['Velocity']))
            # Trim it to 20 entries
            if len(self.velocity_hist) > 20:
                del(self.velocity_hist[0])
            
        elif command == "GAM": #info about the game
            self.s_gameinfo = values
        elif command == "PLR": #another character visible
            # For some reason, this doesn't work in ut2003
            self.s_view_players[values["Id"]] = values
        elif command == "NAV": #a path marker
            # Neither does this
            #print "We have details about a nav point at " + values["Location"]
            self.s_nav_points[values["Id"]] = values
        elif command == "INV": #an object on the ground that can be picked up
            #print values
            self.s_view_items[values["Id"]] = values
        elif command == "FLG": #info about a flag
            #pass these details to the movement behaviour as that stores details of locations etc and may need them
            values["timestamp"] = current_time()
            print "\n".join(["%s=%s" % (k, v) for k, v in values.items()])
            
            self.agent.Movement.receive_flag_details(values)
            # inform the combat behaviour as well
            self.agent.Combat.receive_flag_details(values)
            #print("We have details about a flag.  Its values is: " + values["State"]);
        else:
            pass
    
    def turn(self, degrees):
        utangle = int((degrees * 65535) / 360.0)
        self.send_message("ROTATE", {"Amount" : str(utangle)})
        # self.send_message("TURNTO", {"Pitch" : str(0)})
        
    def get_yaw(self):
        if self.botinfo.has_key("Rotation"):
            return int(re.search(',(.*?),', self.botinfo["Rotation"]).group(1))
        else:
            return None
        
    def get_pitch(self):
        if self.botinfo.has_key("Rotation"):
            return int(re.match('(.*?),', self.botinfo["Rotation"]).group(1))
        else:
            return None
        
    def move(self):
        self.send_message("INCH", {})
        return 1

    # Was the bot hit in the last 2 seconds
    def was_hit(self):
        lsec = 2 # How many seconds to look back to
        isec = 0 # Number of seconds to inhibit consecutive was_hits
        now = current_time()
        def timefilter(item):
            (timestamp, command, value) = item
            if timestamp > now - lsec:
                return 1
            else:
                return 0
        
        # Filter the events from the last lsec seconds
        lastevents = filter(timefilter, self.events)

        if self.hit_timestamp > now - isec:
            # Update the last hit timestamp
            return 0
        else:
            # Update the last hit timestamp
            self.hit_timestamp = now
            if len(lastevents) > 0:
                return 1
            else:
                return 0

    def turning(self):
        # compares the most recent to the leat recent rotation_hist
        # entry. If there is a descrepancy beyond the error fudge,
        # then we say we are rotating
        fudge = 386 # in UT units, roughly 2 degrees
        if len(self.rotation_hist) > 0:
            c_rot = self.rotation_hist[0]
            e_rot = self.rotation_hist[-1]
            diff = abs(c_rot - e_rot)
            if diff > fudge:
                return 1
            
        return 0
        
    def moving(self):
        # If there is recent velocity, return 1
        if len(self.velocity_hist) > 0:
            if self.velocity_hist[0] > 0:
                return 1
        return 0

    def stuck(self):
        # If there is a period of no movement, then return 1
        fudge = 0
        for v in self.velocity_hist:
            if v > fudge:
                return 0
        return 1
        
    def calculate_velocity(self, v):
        (vx, vy, vz) = re.split(',', v)
        vx = float(vx)
        vy = float(vy)
        return utilityfns.find_distance((0,0), (vx, vy))
