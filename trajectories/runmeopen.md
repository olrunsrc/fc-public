Command Arguments

 -x <dest world dx (0.0m)> relative x distance to walk, ideally somewhere beyond the 5 meter mark
 -y <world dy (0.0m)>  relative y distance to translate side to side left is positive the foot in the direction of translation will move first to avoid a collision.  A small (-1mm) negative value will force the right foot to move first. 
 -n <num steps (1)> this will be overridden by a sanity check on maximum stride length These steps are in addition to a half step at the beginning and another at the end which brings the feet back together.
  the stride length is x/n
 -d <dual stance time (1.5s)> aka just stance time
 -s <swing time (1.5s)> swing time
 -l <arm linger (12s)> duration before arm retraction is triggered, from the beginning of the walk
 -r <'-r' to extra gazebo> gazebo will record this run
 -e <extra gazebo args> other gazebo args
 -b <backup dx (-0.0m)> distance to back up before walking, which can be used to adjust where the feet land with respect to the threshold of the door, the time until contact with the button, or just to get a running start
 -w <stance width (0.16m)> change the spacing between the feet.  this is done while backing up.


How it works

Python __main__
parses command line to globals
starts a shell to roslaunch the simulation
calls main()
kills the simulation and exits

main() routine
initializes ROS
creates publishers for footsteps,arms, and chest
creates subscribers for footstep status and robot pose
creates a transform listener
calls monitor() in a 10Hz rate loop
at 28s sim time
creates and publishes a pair of footsteps to back up and/or widen the stance 
at 35s sim time
creates the footstep,arm and chest messages per commandline args
publishes the left arm, right arm and chest trajectories
there is a 5s delay built into these trajectories
at 40s sim time
publishes the footstep data list
loops until monitor() returns done

monitor() routine
clears the screen and prints time, step count, robot x and z, and foot positions at the top
checks the robot x for crossing the start and finish lines and prints the elapsed time
checks the simulation is done
robot x past 5m
robot z less than 0.5m meaning it fell down again
simulation time over 160s


