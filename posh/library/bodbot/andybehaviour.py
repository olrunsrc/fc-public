# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

# POSH Behaviour
from POSH import Behaviour

import random
import re #re is for Regular Expressions
import utilityfns

class AndyBehavior(Behaviour):
    def __init__(self, agent):
        Behaviour.__init__(self, agent,
                           ("stop_bot", "rotate", "big_rotate",
                            "move_player", "pickup_item", "walk"),
                           ("see_player", "see_item", "close_to_player",
                            "hit_object", "fail", "succeed", "is_rotating",
                            "is_walking", "is_stuck"))
        # senses fail and succeed
        self.fail = lambda : False
        self.succeed = lambda : True

    #  == SENSES ==
    
    def see_player(self):
        #print "See a player?"
        # If the bot sees any player, then return 1
        if len(self.agent.Bot.view_players) > 0:
            return 1
        else:
            return 0
            
    def close_to_player(self):
        #print "Checking to see if close to player..."
        # If we are really close to the first player
        closeness = 50
        players = self.agent.Bot.view_players.values()
        
        if len(players) > 0:
            id = players[0]["Id"]
            loc = players[0]["Location"]
            (px, py, pz) = re.split(',', loc)
            (sx, sy, sz) = re.split(',', self.agent.Bot.botinfo["Location"])
            dis = utilityfns.find_distance((float(px),float(py)), (float(sx),float(sy)))
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
        if len(self.agent.Bot.view_items) > 0:
            return 1
        else:
            return 0

    def hit_object(self):
        #print "Was I hit?"
        if self.agent.Bot.was_hit():
            #print "yes!!!"
            return 1
        else:
            #print "Ha, got you"
            return 0

    def is_rotating(self):
        #print "is_rotating"
        return self.agent.Bot.turning()

    def is_walking(self):
        return self.agent.Bot.moving()

    def is_stuck(self):
        print "is stuck?"
        return self.agent.Bot.stuck()
        
    
    #  == ACTIONS ==
    
    def move_player(self):
        #print "Move to player..."
        # Find the first player and move to it
        players = self.agent.Bot.view_players.values()
        
        if len(players) > 0:
            id = players[0]["Id"]
        #    print str(id)
            self.agent.Bot.send_message("RUNTO", {"Target" : str(id)})
            
        return 1

    def pickup_item(self):
        print "Pickup item..."
        # Pickup the first item on the list
        items = self.agent.Bot.view_items.values()
        
        if len(items) > 0:
            id = items[0]["Id"]           
            self.agent.Bot.send_message("RUNTO", {"Target": str(id)})
        return 1

    def rotate(self, angle = None):
        #print "Rotating..."
        def turnleft():
            self.agent.Bot.turn(90)
            
        def turnright():
            self.agent.Bot.turn(-90)
            
        actions = (turnleft, turnright)
        if angle == None:
            random.choice(actions)() # Run the action randomly
        else:
            self.agent.Bot.turn(angle)
            # was self.agent.Bot.move(angle) but this is silly
        return 1
        
    # rotates by 160 degrees.  Should allow quick exploring of places without getting stuck at edges etc
    def big_rotate(self):
        print "big rotate"
        angle = 160
        self.rotate(angle)
        return 1

    def walk(self):
        #print "Walking..."
        return self.agent.Bot.move()

    def stop_bot(self):
        #print "Stopping Bot"
        self.agent.Bot.send_message("STOP", {})
        return 1
