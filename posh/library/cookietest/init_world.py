"""Default world initialisation script.

This script is only present in cookietest for demonstration purposes. The only thing it does
is print the arguments that cookietest was given.
"""

print "cookietest world initialisation script called with arguments"
print "'%s'" % str(world.args())
# could return the world object with
#world.set(world_obj)
# but we don't need any
