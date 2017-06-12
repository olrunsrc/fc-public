# Jython compatibility
from __future__ import nested_scopes
from POSH.jython_compat import *

from POSH import Behaviour
from POSH.utils import current_time

import utilityfns

class Movement(Behaviour):
    def __init__(self, agent):
        Behaviour.__init__(self, agent,
                           ("walk_to_nav_point", "to_enemy_flag", "to_own_base",
                            "to_own_flag", "to_enemy_base", "inch",
                            "runto_medical_kit", "runto_weapon"),
                           ("at_enemy_base", "at_own_base", "know_enemy_base_pos",
                            "know_own_base_pos", "reachable_nav_point",
                            "enemy_flag_reachable", "our_flag_reachable",
                            "see_enemy", "see_reachable_medical_kit",
                            "see_reachable_weapon", "too_close_for_path"))
        self.PosInfo = PositionsInfo()
        # set up useful constants
        self.PathHomeID = "PathHome"
        self.ReachPathHomeID = "ReachPathHome"
        self.PathToEnemyBaseID = "PathThere"
        self.ReachPathToEnemyBaseID = "ReachPathThere"
    
    # === SENSES ===
    
    def at_enemy_base(self):
        #print "in at_enemy_base sense"
        if not self.agent.Bot.botinfo.has_key("Location"):
            return 0
        LocTuple = utilityfns.location_string_to_tuple(self.agent.Bot.botinfo["Location"])
        
        if self.PosInfo.EnemyBasePos == None:
            return 0
        else:
            (SX, SY, SZ) = LocTuple
            (EX, EY, EZ) = utilityfns.location_string_to_tuple(self.PosInfo.EnemyBasePos)
            if utilityfns.find_distance((SX, SY), (EX, EY)) < 100: # this distance may need adjusting in future (we may also wish to consider the Z axis)
                return 1
            else:
                return 0
                
    # returns 1 if we're near enough to our own base
    def at_own_base(self):
        if not self.agent.Bot.botinfo.has_key("Location"):
            return 0
        LocTuple = utilityfns.location_string_to_tuple(self.agent.Bot.botinfo["Location"])
        
        if self.PosInfo.OwnBasePos == None:
            return 0
        else:
            (SX, SY, SZ) = LocTuple
            (HX, HY, HZ) = utilityfns.location_string_to_tuple(self.PosInfo.OwnBasePos)
            if utilityfns.find_distance((HX, HY), (SX, SY)) < 10: # this distance may need adjusting in future (we may also wish to consider the Z axis)
                return 1
            else:
                return 0
    
    # returns 1 if we have a location for the enemy base
    def know_enemy_base_pos(self):
        #print "in know_enemy_base_pos sense"
        if self.PosInfo.EnemyBasePos == None:
            return 0
        else:
            return 1
            
    # returns 1 if we have a location for our own base
    def know_own_base_pos(self):
        if self.PosInfo.OwnBasePos == None:
            return 0
        else:
            return 1
            
    # returns 1 if there's a reachable nav point in the bot's list which we're not already at
    def reachable_nav_point(self):
        #print "reachable_nav_point?"
        
        # setup location tuple
        if not self.agent.Bot.botinfo.has_key("Location"):
            # if we don't know where we are, treat it as (0,0,0) as that will just mean we go to the nav point even if we're close by
            (SX, SY, SZ) = (0, 0, 0)
        else:
            (SX, SY, SZ) = utilityfns.location_string_to_tuple(self.agent.Bot.botinfo["Location"])
        
        print str(SX) + str(SY) + str(SZ)
        
        # is there already a navpoint we're aiming for?
        DistanceTolerance = 30 # how near we must be to be thought of as at the nav point
        if self.PosInfo.ChosenNavPoint != None:
            #print "  One already chosen.  DistanceTolerance is", DistanceTolerance
            (NX, NY, NZ) = self.PosInfo.ChosenNavPoint
            if utilityfns.find_distance((NX, NY), (SX, SY)) > DistanceTolerance:
                #print "    Distance Tolerance check passed (i.e. far away enough)"
                return 1
            else:
                self.PosInfo.VisitedNavPoints.append((NX, NY, NZ)) # set this NP as visited
                self.PosInfo.ChosenNavPoint = None # the visited check override isn't used here, instead just clear it
        
        # now look at the list of navpoints the bot can see
        if self.agent.Bot.nav_points == None or len(self.agent.Bot.nav_points) == 0:
            print "  no nav points"
            return 0
        else:
            #print "  else -- work through nav points"
            # nav_points is a list of tuples.  Each tuple contains an ID and a dictionary of attributes as defined in the API
            # Search for reachable nav points
            PossibleNPs = self.get_reachable_nav_points(self.agent.Bot.nav_points.items(), DistanceTolerance, (SX, SY, SZ))
            
            # now work through this list of NavPoints until we find once that we haven't been to
            # or the one we've been to least often
            if len(PossibleNPs) == 0:
                print "    No PossibleNPs"
                return 0 # nothing found
            else:
                self.PosInfo.ChosenNavPoint = self.get_least_often_visited_navpoint(PossibleNPs)
                print "    Possible NP, returning 1"
                return 1
            
    def get_least_often_visited_navpoint(self, PossibleNPs):
        CurrentMin = self.PosInfo.VisitedNavPoints.count(PossibleNPs[0])
        CurrentMinNP = PossibleNPs[0]
        for CurrentNPTuple in PossibleNPs:
            CurrentCount = self.PosInfo.VisitedNavPoints.count(CurrentNPTuple)
            if CurrentCount < CurrentMinNP:
                CurrentMin = CurrentCount
                CurrentMinNP = CurrentNPTuple
        return CurrentMinNP
        
    # also performs a distance tolerance check, (SX, SY, SZ) is position of player
    def get_reachable_nav_points(self, NPList, DistanceTolerance, (SX, SY, SZ)):
        PossibleNPs = []
        for CurrentNP in NPList:
            #print type(CurrentNP)
            #print " is the type\n"
            #print type(CurrentNP[1])
            #print " is the type of its 2nd element\n"
            (NX, NY, NZ) = utilityfns.location_string_to_tuple((CurrentNP[1])["Location"])
            if CurrentNP[1]["Reachable"] == "True" and utilityfns.find_distance((NX, NY), (SX, SY)) > DistanceTolerance:
                PossibleNPs.append((NX, NY, NZ))
        return PossibleNPs
            
    # returns 1 if the enemy flag is specified as reachable
    def enemy_flag_reachable(self):
        if self.PosInfo.has_enemy_flag_info_expired():
            self.PosInfo.expire_enemy_flag_info()
            
        #debug
        #print "in enemy_flag_reachable"
        
        #if self.PosInfo.EnemyFlagInfo != {}:
        #    print self.PosInfo.EnemyFlagInfo
        
        # Made simpler FA
        if self.PosInfo.EnemyFlagInfo != {} and self.PosInfo.EnemyFlagInfo["Reachable"] == "True":
            return 1
        return 0
            
    def our_flag_reachable(self):
        print "in our_flag_reachable"
        
        if self.PosInfo.has_our_flag_info_expired():
            self.PosInfo.expire_our_flag_info()
        
        if self.PosInfo.OurFlagInfo == {}:
            return 0
        elif self.PosInfo.OurFlagInfo["Reachable"] == "True":
            print "  is reachable!"
            return 1
        return 0
        
    def see_enemy(self):
        if len(self.agent.Bot.view_players) == 0 or self.agent.Bot.botinfo == {}: #if botinfo is {}, we can't yet set anything
            return 0
        else:
            # work through, looking for an enemy
            OurTeam = self.agent.Bot.botinfo["Team"]
            Players = self.agent.Bot.view_players.values()
            for CurrentPlayer in Players:
                #print CurrentPlayer
                if CurrentPlayer["Team"] != OurTeam:
                    print "we can see an enemy!"
                    return 1
        return 0
        
    def see_reachable_medical_kit(self):
        if len(self.agent.Bot.view_items) < 1:
            return 0
        else:
            # look through for a medical kit
            ItemValues = self.agent.Bot.view_items.values()
            for CurrentItem in ItemValues:
                if (CurrentItem["Class"].find("Health") != -1 or CurrentItem["Class"].find("MedBox")) and CurrentItem["Reachable"] == "True":
                    return 1
            return 0
            
    def see_reachable_weapon(self):
        if len(self.agent.Bot.view_items) < 1:
            return 0
        else:
            # look through for a weapon
            ItemValues = self.agent.Bot.view_items.values()
            for CurrentItem in ItemValues:
                if utilityfns.is_known_weapon_class(CurrentItem["Class"]) and CurrentItem["Reachable"] == "True":
                    return 1
            return 0
            
    # see PositionsInfo class for comments on TooCloseForPath
    def too_close_for_path(self):        
        if self.PosInfo.has_too_close_for_path_expired():
            self.PosInfo.expire_too_close_for_path()
    
        if self.PosInfo.TooCloseForPath:
            print "we are too close for path"
        return self.PosInfo.TooCloseForPath
                
    # === ACTIONS ===
    
    def runto_medical_kit(self):
        if len(self.agent.Bot.view_items) < 1:
            return 1
        else:
            # look through for a medical kit
            ItemValues = self.agent.Bot.view_items.values()
            for CurrentItem in ItemValues:
                if (CurrentItem["Class"].find("Health") != -1 or CurrentItem["Class"].find("MedBox")) and CurrentItem["Reachable"] == "True":
                    self.send_runto_or_strafe_to_location(CurrentItem["Location"])
            return 1
            
    def runto_weapon(self):
        if len(self.agent.Bot.view_items) < 1:
            return 1
        else:
            # look through for a weapon
            ItemValues = self.agent.Bot.view_items.values()
            for CurrentItem in ItemValues:
                if utilityfns.is_known_weapon_class(CurrentItem["Class"]) and CurrentItem["Reachable"] == "True":
                    print "runto weapon",
                    print CurrentItem["Class"]
                    self.send_runto_or_strafe_to_location(CurrentItem["Location"])
                    return 1
            return 1
    
    # Runs to the ChosenNavPoint
    def walk_to_nav_point(self):
        #print "walk_to_nav_point: " + utilityfns.location_tuple_to_string(self.PosInfo.ChosenNavPoint)
        
        # have we already sent it?
        #if not utilityfns.is_previous_message(self.agent.Bot, ("RUNTO", {"Location" : utilityfns.location_tuple_to_string(self.PosInfo.ChosenNavPoint)})):
        #    self.agent.Bot.send_message("RUNTO", {"Location" : utilityfns.location_tuple_to_string(self.PosInfo.ChosenNavPoint)})
            #print "sending message"
        #else:
            #print "already sent"
            
        # new version just calls the utility function
        #utilityfns.send_if_not_prev(self.agent.Bot, ("RUNTO", {"Location" : utilityfns.location_tuple_to_string(self.PosInfo.ChosenNavPoint)}))
        
        #even newer version has a strafe check
        self.send_runto_or_strafe_to_location(utilityfns.location_tuple_to_string(self.PosInfo.ChosenNavPoint))
        
        return 1
            
    # runs to the enemy flag
    def to_enemy_flag(self):
        print "!!in to_enemy_flag"
        print "\n".join(["%s=%s" % (k, v) for k, v in self.PosInfo.EnemyFlagInfo.items()])
        
        if self.PosInfo.has_enemy_flag_info_expired():
            self.PosInfo.expire_enemy_flag_info()
        
        if self.PosInfo.EnemyFlagInfo != {}:
            self.agent.Bot.send_message("RUNTO", {"Target" : self.PosInfo.EnemyFlagInfo["Id"]})
        return 1
            
    def to_own_flag(self):  
        if self.PosInfo.has_our_flag_info_expired():
            self.PosInfo.expire_our_flag_info()
    
        if self.PosInfo.OurFlagInfo != {}:
            # was self.agent.Bot.send_message("RUNTO", {"Location" : self.PosInfo.OurFlagInfo["Location"]})
            self.send_runto_or_strafe_to_location(self.PosInfo.OurFlagInfo["Location"])
        return 1
            
    # runs to the bot's own base by getting a list of navpoints showing the way there
    def to_own_base(self):
        print "to_own_base"
        DistanceTolerance = 30
        # If we don't know where our own base is, then do nothing
        # However, this action should never fire unless we do know where our base is
        if self.PosInfo.OwnBasePos == None:
            print "Don't know where own base is!"
            return 1
        
        def send_getpath():
            print "in send_getpath"
            if not utilityfns.is_previous_message(self.agent.Bot, ("GETPATH", {"Location" : self.PosInfo.OwnBasePos, "Id" : self.PathHomeID})):
                self.agent.Bot.send_message("GETPATH", {"Location" : self.PosInfo.OwnBasePos, "Id" : self.PathHomeID}) # the ID allows us to match requests with answers
                print "sent GETPATH"
            else:
                print "GETPATH already sent"
        
        # if we haven't already got a list of path nodes to follow then send the GETPATH message
        # to try and mitigate the problem of pathhome being cleared part way through this, we assign
        # the relevant value to a variable and then use that throughout, so the array is checked as infrequently as possible
        # it's not an ideal fix though!
        if self.PosInfo.PathHome == []:
            send_getpath()
        else:
            if not self.to_known_location(self.PosInfo.PathHome, DistanceTolerance):
                print "DT check failed, tailing"
                self.PosInfo.PathHome = utilityfns.tail(self.PosInfo.PathHome)
                if self.PosInfo.PathHome != []:
                    print "tail not empty"
                    PathLoc = self.PosInfo.PathHome[0]
                    self.send_runto_or_strafe_to_location(PathLoc)
                else:
                    send_getpath()
    
        #before we return, send a checkreach command about the current navpoint.  That way the list can be recreated if it becomes incorrect
        if self.PosInfo.PathHome != [] and self.PosInfo.PathHome != None:
            self.agent.Bot.send_message("CHECKREACH", {"Location" : self.PosInfo.PathHome[0], "Id" : self.ReachPathHomeID, "From" : self.agent.Bot.botinfo["Location"]})
        print "about to return from to_own_base"
    
    # runs to the enemy's base by getting a list of navpoints showing the way there
    def to_enemy_base(self):
        print "to_enemy_base"
        DistanceTolerance = 30
        # If we don't know where the base is, then do nothing
        # However, this action should never fire unless we do know where it is
        if self.PosInfo.EnemyBasePos == None:
            print "Don't know where enemy base is!"
            return 1
        
        def send_getpath():
            print "in send_getpath"
            utilityfns.send_if_not_prev(self.agent.Bot, ("GETPATH", {"Location" : self.PosInfo.EnemyBasePos, "Id" : self.PathToEnemyBaseID}))
        
        # if we haven't already got a list of path nodes to follow then send the GETPATH message
        # to try and mitigate the problem of pathhome being cleared part way through this, we assign
        # the relevant value to a variable and then use that throughout, so the array is checked as infrequently as possible
        # it's not an ideal fix though!
        if self.PosInfo.PathToEnemyBase == []:
            send_getpath()
        else:
            if not self.to_known_location(self.PosInfo.PathToEnemyBase, DistanceTolerance):
                print "DT check failed, tailing"
                self.PosInfo.PathToEnemyBase = utilityfns.tail(self.PosInfo.PathToEnemyBase)
                if self.PosInfo.PathToEnemyBase != []:
                    print "tail not empty"
                    PathLoc = self.PosInfo.PathToEnemyBase[0]
                    self.send_runto_or_strafe_to_location(PathLoc)
                else:
                    send_getpath()
    
        #before we return, send a checkreach command about the current navpoint.  That way the list can be recreated if it becomes incorrect
        if self.PosInfo.PathToEnemyBase != [] and self.PosInfo.PathToEnemyBase != None:
            self.agent.Bot.send_message("CHECKREACH", {"Location" : self.PosInfo.PathToEnemyBase[0], "Id" : self.ReachPathToEnemyBaseID, "From" : self.agent.Bot.botinfo["Location"]})
        print "about to return from to_enemy_base"
    
    # returns 1 and sends a runto message for the provided location if the DistanceTolerance check passes
    # otherwise returns 0
    def to_known_location(self, Location, DistanceTolerance):
        if len(Location) == 0:
            return 1 # even though we failed, we return 1 so that it doesn't tail the list
        # to first point in current list, if we're not already there
        Location0 = Location[0]
        (HX, HY, HZ) = utilityfns.location_string_to_tuple(Location0)
        (SX, SY, SZ) = utilityfns.location_string_to_tuple(self.agent.Bot.botinfo["Location"])
        if utilityfns.find_distance((HX, HY), (SX, SY)) > DistanceTolerance:
            print "DistanceTolerance check passed"
            
            print "About to send RUNTO to",
            print Location0
            print "Current location",
            print self.agent.Bot.botinfo["Location"]
            
            self.send_runto_or_strafe_to_location(Location0, 0)
            # was
            #if not utilityfns.is_previous_message(self.agent.Bot, ("RUNTO", {"Location" : PathLoc})):
            #    self.agent.Bot.send_message("RUNTO", {"Location" : PathLoc})
            #    print "Running to " + PathLoc
            return 1
        else:
            return 0
        
    #not used at present (18/2/2005)
    def inch(self):
        # just add a bit to the x value
        print "in inch"
        (SX, SY, SZ) = utilityfns.location_string_to_tuple(self.agent.Bot.botinfo["Location"])
        NewLocTuple = (SX + 150, SY, SZ)
        self.send_runto_or_strafe_to_location(utilityfns.location_tuple_to_string(NewLocTuple))
  
    # just because something was reachable the last time we knew about it doesn't mean it still is
    #def expire_reachable_info(self):
    #    if self.PosInfo.OurFlagInfo != {} and self.PosInfo.OurFlagInfo.has_key("Reachable"):
    #        self.PosInfo.OurFlagInfo["Reachable"] = "0"
    #    if self.PosInfo.EnemyFlagInfo != {} and self.PosInfo.EnemyFlagInfo.has_key("Reachable"):
    #        self.PosInfo.EnemyFlagInfo["Reachable"] = "0"
    #    
    #    self.PosInfo.TooCloseForPath = 0
    
    # === OTHER FUNCTIONS ===
    
    # checks the previous sent message against the provided one, returning 1 if they match
    # now replaced by is_previous_message(bot, Msg) in utilityfns
    #def is_previous_message(self, Msg):
    #    if self.agent.Bot.sent_msg_log == None or \
    #    len(self.agent.Bot.sent_msg_log) == 0 or \
    #    self.agent.Bot.sent_msg_log[-1] != Msg:
    #        return 0
    #    return 1
    
    # updates the flag positions in PositionsInfo
    # also updates details of bases, if relevant info sent
    # the position of a flag is how we determine where the bases are
    def receive_flag_details(self, values):
        #print "f",
        #print values["Reachable"]
        print 'in receive_flag_details'
        print "\n".join(["%s=%s" % (k, v) for k, v in values.items()])
        if self.agent.Bot.botinfo == {}: #if botinfo is {}, we can't yet set anything
            return
        
        #print "in receive_flag_details.  Values are:"
        #print values
        
        #set flag stuff
        OurTeam = self.agent.Bot.botinfo["Team"]
        if values["Team"] == OurTeam:
            self.PosInfo.OurFlagInfo = values
            #print "our flag"
        else:
            self.PosInfo.EnemyFlagInfo = values
            #print "enemy flag"
        
        # now set base stuff if appliable
        if values["State"] == "home":
            if values["Team"] == self.agent.Bot.botinfo["Team"]:
                self.PosInfo.OwnBasePos = values["Location"]
            else:
                self.PosInfo.EnemyBasePos = values["Location"]
                #print "enemy base at",
                #print self.PosInfo.EnemyBasePos
                #print "self.PosInfo.EnemyBasePos has type",
                #print type(self.PosInfo.EnemyBasePos)
    
    # if the 'ID' key is PathHome then it tells the bot how to get home.
    # we need to turn the dictionary into a list, ordered by key ('0' ... 'n')
    # at present other IDs are ignored
    def receive_pth_details(self, ValuesDict):
        if not ValuesDict.has_key("ID"):
            return
        elif ValuesDict["ID"] == self.PathHomeID:
            self.PosInfo.PathHome =  utilityfns.nav_point_dict_to_ordered_list(ValuesDict)
        elif ValuesDict["ID"] == self.PathToEnemyBaseID:
            self.PosInfo.PathToEnemyBase =  utilityfns.nav_point_dict_to_ordered_list(ValuesDict)
            
        # if there's no 0 key we're being given an empty path, so set TooCloseForPath to the current time
        # so that we can check how old the information is later on
        if not ValuesDict.has_key("0"):
            self.PosInfo.TooCloseForPath = current_time()
        else:
            self.PosInfo.TooCloseForPath = 0
            
    
    # used in validating the bot's path home or to the enemy flag
    # if the thing has the right ID, then clear the relevant path if it's not reachable
    def receive_rch_details(self, ValuesDict):
        print "in receive_rch_details"
        if not ValuesDict.has_key("ID"):
            return
        elif ValuesDict["ID"] == self.ReachPathHomeID and ValuesDict["Reachable"] == "False":
            self.PosInfo.PathHome = []
            print "Cleared PathHome"
        elif ValuesDict["ID"] == self.ReachPathToEnemyBaseID and ValuesDict["Reachable"] == "False":
            self.PosInfo.PathToEnemyBase = []
            print "Cleared PathToEnemyBase"
        
    # if the combatinfo class specifies that we need to remain focused on a player, send a relevant strafe command
    # to move to the provided location.  Otherwise, a runto
    def send_runto_or_strafe_to_location(self, Location, PerformPrevCheck = 1):
        #expire focus id info if necessary FA 
        if self.agent.Combat.CombatInfo.KeepFocusOnID != None and self.agent.Combat.CombatInfo.has_focus_id_expired(): 
            self.agent.Combat.CombatInfo.expire_focus_id()

            # Seems out of place, need to find a better way of doing this FA
            self.agent.Bot.send_message("STOPSHOOT", {}) # no-one to focus on
    
        if self.agent.Combat.CombatInfo.KeepFocusOnID != None:
            Message = ("STRAFE", {"Location" : Location, "Target": self.agent.Combat.CombatInfo.KeepFocusOnID})
            if PerformPrevCheck:
                utilityfns.send_if_not_prev(self.agent.Bot, Message)
            else:
                self.agent.Bot.send_message(Message[0], Message[1])
            
        else:
            Message = ("RUNTO", {"Location" : Location})
            if PerformPrevCheck:
                utilityfns.send_if_not_prev(self.agent.Bot, Message)
            else:
                self.agent.Bot.send_message(Message[0], Message[1])
                print "have just sent",
                print Message
                
    # clean-up after dying
    def receive_die_details(self, ValuesDict):
        self.PosInfo.PathHome = []
        self.PosInfo.PathToEnemyBase = []
        self.PosInfo.VisitedNavPoints = [] # this is new
        self.PosInfo.OurFlagInfo = {}
        self.PosInfo.EnemyFlagInfo = {}
                
    
# This class stores details about where things are
# More stuff will be added in future
class PositionsInfo:
    def __init__(self):
        self.OwnBasePos = None
        self.EnemyBasePos = None
        self.VisitedNavPoints = []
        self.ChosenNavPoint = None
        self.OurFlagInfo = {}
        self.EnemyFlagInfo = {}
        
        # a list of nav points showing the way to various places
        self.PathHome = []
        self.PathToEnemyBase = []
        self.TooCloseForPath = 0 # set to current time if we're sent a blank path.  Blank paths indicate that we're right next to something but can't actually see it

    # FA
    def has_enemy_flag_info_expired(self, lsecs = 10):
        if self.EnemyFlagInfo != {} and self.EnemyFlagInfo.has_key("Reachable"):
            if self.EnemyFlagInfo["timestamp"] < current_time() - lsecs:
                return 1
        return 0
    
    # Have to call check_enemy_flag_info_expired before calling this FA    
    def expire_enemy_flag_info(self):
        self.EnemyFlagInfo["Reachable"] = "0"
    
    # FA
    def has_our_flag_info_expired(self, lsecs = 10):
        if self.OurFlagInfo != {} and self.OurFlagInfo.has_key("Reachable"):
            if self.OurFlagInfo["timestamp"] < current_time() - lsecs:
                return 1
        return 0
    
    # Have to call check_our_flag_info_expired before calling this FA    
    def expire_our_flag_info(self):
        self.OurFlagInfo["Reachable"] = "0"
    
    # FA
    def has_too_close_for_path_expired(self, lsecs = 10):
        if self.TooCloseForPath < current_time() - lsecs:
            return 1
        return 0
    
    # FA
    def expire_too_close_for_path(self):
        self.TooCloseForPath = 0
