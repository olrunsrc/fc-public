from __future__ import nested_scopes
import string

# Some utility functions
def find_distance(one, two):
    (x1, y1) = one
    (x2, y2) = two
    return ((((x1-x2)**2) + ((y1-y2)**2))**0.5)
    
# takes a string of the form 'x,y,z' and converts it to a tuple (x,y,z)
def location_string_to_tuple(LocationString):
    LocList = string.split(LocationString, ",")
    if len(LocList) != 3:
        return (0,0,0)
    LTuple = (float(LocList[0]), float(LocList[1]), float(LocList[2]))
    return LTuple
    
def location_tuple_to_string(LocationTuple):
    LString = str(LocationTuple[0]) + "," + str(LocationTuple[1]) + "," + str(LocationTuple[2])
    return LString
    
# returns negative if the number a represents is < the number b represents.  0 if equal, positive if >
def compare_number_strings(a, b):
    anum = int(a)
    bnum = int(b)
    if anum < bnum:
        return -1
    elif anum == bnum:
        return 0
    else:
        return 1
    
# lists of nav points arrive as dicts with an "ID" key and keys "0", "1", .... "n" these need converting to lists
def nav_point_dict_to_ordered_list(ValuesDict):
    del ValuesDict["ID"] #remove the ID key to leave just numbers
    #now get a list of just keys, and sort it to use in extracting the key:value pairs
    KeyList = ValuesDict.keys()
    
    # debug
    if ValuesDict.has_key("Reachable"):
        print ValuesDict
        print "-------"
    
    KeyList.sort(compare_number_strings) #need a home-grown sort function as although they're strings, we don't want "10" < "2"
    
    #now use the keylist to create an ordered list of location strings
    LocList = []
    CurrentLoc = 0
    while CurrentLoc < len(ValuesDict):
        #need to strip out the ID by including only everything after the first space 
        LocString = ValuesDict[str(CurrentLoc)]
        LocString = LocString[string.find(LocString, " ") : len(LocString)]
        LocString = LocString.strip()
        LocList.append(LocString)
        CurrentLoc = CurrentLoc+1
    
    return LocList

def tail(SentSequence):
    if SentSequence == [] or len(SentSequence) == 1:
        return []
    else:
        return SentSequence[1 : len(SentSequence)-1]
        
# checks the bot's previous sent message against the provided one, returning 1e if they match
def is_previous_message(bot, Msg):
    if bot.sent_msg_log == None or \
    len(bot.sent_msg_log) == 0 or \
    bot.sent_msg_log[-1] != Msg:
        return 0
        return 1
        
def send_if_not_prev(bot, Msg):
    if not is_previous_message(bot, Msg):
        bot.send_message(Msg[0], Msg[1])
        
def is_known_weapon_class(SentClass):
    if SentClass == None:
        return 0
    else:
        if SentClass.find("goowand") != -1:
            return 1
    return 0
