#!/usr/bin/python
#
#  Gamebots Example
#
#  We need to start a comms thread in order to get updates
#  to the agent status from the server.
#
# WARNING: This behaviour has not been updated with the latest refactoring of
# the POSH implementation. So it won't work. (29/07/08)
from __future__ import nested_scopes
from socket import *
from POSH.basic import Base
from POSH import posh_utils
import re
import thread
import random

# Init world in this example connects to gamebots server
def init_world(*args, **kw):
    pass
    

# Returns the behavior object
def make_behavior(ip, port, botname, agent, *args, **kw):
    bot = Bot_Agent(agent, ip, port, botname)
    b = Behavior(agent = agent)
    b.bind_bot(bot)
    b.bot.connect()
    return [b]

# Called when pyposh is shutting down
def destroy_world():
    pass

# Some utility functions
def find_distance(one, two):
    (x1, y1) = one
    (x2, y2) = two
    return ((((x1-x2)**2) + ((y1-y2)**2))**0.5)

# Keeps a local copy of the bot state. Gamebots does not support
# queries on the agent sense, it sends a copy of the environment
# to the agent periodically.
class Bot_Agent(Base):
    def __init__(self, agent, ip, port, botname):
        Base.__init__(self, agent)
        self.ip = ip
        self.port = port
        self.botname = botname
        self.events = []
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
        self.msg_log = [] # Temp Log
        self.msg_log_max = 4096 # Max Temp Log size
        self.hit_timestamp = 0 # Used to inhibit was_hit()
        self.thread_active = 0
        self.kill_connection = 0
        self.rotation_hist = []
        self.velocity_hist = []
        self.thread_active = 0
        self.conn_ready = 0
        self.conn_thread_id = None
                             

    def proc_item(self, string):
        (cmd, varstring) = re.compile('\s+').split(string, 1)
        vars = re.compile('\{(.*?)\}').findall(varstring)
        var_dict = {}
        for var in vars:
            (attr, value) = re.compile('\s+').split(var, 1)
            var_dict[attr] = value
        return (cmd, var_dict)

    # Calls connect_thread in a new thread
    def connect(self):
        self.log.info("Connecting to Server")
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
            self.log.info("Connected to server")
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
                self.send_message("INIT", {"Name" : self.botname})
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
            
            # print "R>> " + x
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
                # When a sync batch ends, we cant to make the shadow
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
                self.events.append((posh_utils.current_time(), cmd, dict))
            elif cmd == "SEE":
                # Update the player positions
                self.view_players[dict["Id"]] = dict
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
        for (attr, value) in dict.items():
            string = string + " {" + attr + " " + value + "}"
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

    def proc_sync(self, command, values):
        if command == "SLF":
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
            
        elif command == "GAM":
            self.s_gameinfo = values
        elif command == "PLR":
            # For some reason, this doesn't work in ut2003
            self.s_view_players[values["Id"]] = values
        elif command == "NAV":
            # Neither does this
            self.s_nav_points[values["Id"]] = values
        elif command == "INV":
            self.s_view_items[values["Id"]] = values
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
        now = posh_utils.current_time()
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
        return find_distance((0,0), (vx, vy))

#
# The behavior class, we can merge this with the bot class
# if we really wanted too, but keep it separate for now so that
# we can someday rip out the gamebots class for other stuff.

class Behavior(Base):
    def __init__(self, **kw):
        Base.__init__(self, **kw) # Call the ancestor init
        self.act_dict = {}
        self.sense_dict = {}
        self.init_acts()
        self.init_senses()
        # These are behavior varibles

    # This method is called by agent.execute in posh_agent to make sure
    # that the behavior is ok every cycle. Returns 0 if everything is OK.
    # We can assign error codes or something similar.
    def check_error(self):
        if self.bot.conn_ready: # Check the bot to see if connection
            return 0
        else:
            return 1
        
    # The agent has recieved a request for exit. Stop running everything.
    def exit_prepare(self):
        self.bot.disconnect()

    def init_acts(self):
        self.add_act("stop-bot", self.stop_bot)
        self.add_act("rotate", self.rotate)
        self.add_act("move-player", self.move_player)
        self.add_act("pickup-item", self.pickup_item)
        self.add_act("walk", self.walk)

    def init_senses(self):
        self.add_sense("see-player", self.see_player)
        self.add_sense("see-item", self.see_item)
        self.add_sense("close-to-player", self.close_to_player)
        self.add_sense("hit-object", self.was_hit)
        self.add_sense("fail", lambda : 0)
        self.add_sense("succeed", lambda : 1)
        self.add_sense("is-rotating", self.is_rotating)
        self.add_sense("is-walking", self.is_walking)
        self.add_sense("is-stuck", self.is_stuck)

#    def add_act(self, name, act):
#        self.act_dict[name] = act
#
#    def add_sense(self, name, sense):
#        self.sense_dict[name] = sense
#
#    def get_act(self, name):
#        if self.act_dict.has_key(name):
#            return self.act_dict[name]
#        else:
#            return None
#        
#    def get_sense(self, name):
#        if self.sense_dict.has_key(name):
#            return self.sense_dict[name]
#        else:
#            return None


    def add_act(self, name, act):
        self.agent.act_dict[name] = act

    def add_sense(self, name, sense):
        self.agent.sense_dict[name] = sense

    def get_act(self, name):
        if self.agent.act_dict.has_key(name):
            return self.agent.act_dict[name]
        else:
            return None
        
    def get_sense(self, name):
        if self.agent.sense_dict.has_key(name):
            return self.agent.sense_dict[name]
        else:
            return None

    def bind_bot(self, bot):
        self.bot = bot
        bot.agent = self.agent
        
    def see_player(self):
        # print "See a player?"
        # If the bot see any player, then return 1
        if len(self.bot.view_players) > 0:
            return 1
        else:
            return 0

    def move_player(self):
        #print "Move to player..."
        # Find the first player and move to it
        players = self.bot.view_players.values()
        
        if len(players) > 0:
            id = players[0]["Id"]
        #    print str(id)
            self.bot.send_message("RUNTO", {"Target" : str(id)})
            
        return 1
        

    def close_to_player(self):
        #print "Checking to see if close to player..."
        # If we are really close to the first player
        closeness = 50
        players = self.bot.view_players.values()
        
        if len(players) > 0:
            id = players[0]["Id"]
            loc = players[0]["Location"]
            (px, py, pz) = re.split(',', loc)
            (sx, sy, sz) = re.split(',', self.bot.botinfo["Location"])
            dis = find_distance((float(px),float(py)), (float(sx),float(sy)))
            # print dis
            if dis < closeness:
                return 1
            else:
                return 0
        else:
            return 0
        

    def see_item(self):
        #print "See an item?"
        # If we see an item, then return 1
        if len(self.bot.view_items) > 0:
            return 1
        else:
            return 0

    def pickup_item(self):
        #print "Pickup item..."
        # Pickup the first item on the list
        items = self.bot.view_items.values()
        
        if len(items) > 0:
            id = items[0]["Id"]           
            self.bot.send_message("RUNTO", {"Target": str(id)})
        return 1

    def rotate(self, angle = None):
        #print "Rotating..."
        def turnleft():
            self.bot.turn(90)
            
        def turnright():
            self.bot.turn(-90)
            
        actions = (turnleft, turnright)
        if angle == None:
            random.choice(actions)() # Run the action randomly
        else:
            self.bot.move(angle)
        return 1

    def walk(self):
        #print "Walking..."
        return self.bot.move()

    def stop_bot(self):
        #print "Stopping Bot"
        self.bot.send_message("STOP", {})
        return 1

    def was_hit(self):
        #print "Was I hit?"
        if self.bot.was_hit():
            #print "yes!!!"
            return 1
        else:
            #print "Ha, got you"
            return 0

    def is_rotating(self):
        return self.bot.turning()

    def is_walking(self):
        return self.bot.moving()

    def is_stuck(self):
        return self.bot.stuck()



#    def see_cookie(self):
#        dirlist = os.listdir(os.getcwd())
#        for x in dirlist:
#            if x == "cookie":
#                print "Fount cookie at " + os.getcwd()
#                return 1
#        print "Cookie Not Found at " + os.getcwd()
#        return 0

#    def change_dir(self):
#        tmplist = os.listdir(os.getcwd())
#        dirlist = []

#        for x in tmplist:
#            if os.path.isdir(x):
#                dirlist.append(x)
#        if os.getcwd() != "/home/andy/pyposh":
#            dirlist.append("..")

#        os.chdir(dirlist[random.randrange(len(dirlist))])
#        print "Looking for cookie in " + os.getcwd()
#        return 1
