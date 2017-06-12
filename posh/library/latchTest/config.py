time_steps_to_death=2000
time_steps_at_latch=100

common_decrement=0.1
common_increment=1.1
common_move_distance=2

common_lower=time_steps_to_death*common_decrement
common_upper=common_lower+(time_steps_at_latch/(common_increment-common_decrement))
common_latch_threshold=None #common_lower

common_rnd_lower=int(common_lower) #+(common_lower*0.05))
common_rnd_upper=int(common_lower) #+(common_lower*0.15))

#interruptions caused by class interrupt
dr_num_interrupts=0
eat_num_interrupts=0

############### Drinking ##################
dr_lower_threshold=common_lower
dr_upper_threshold=common_upper
dr_latch_threshold=common_latch_threshold
dr_rnd_lower=common_rnd_lower
dr_rnd_upper=common_rnd_upper
dr_increment=common_increment
dr_decrement=common_decrement
dr_registered=('a_drink','a_move_to_drink','a_pick_drink')
dr_move_distance=common_move_distance

###############  Eating  ##################
eat_lower_threshold=common_lower
eat_upper_threshold=common_upper 
eat_latch_threshold=common_latch_threshold
eat_rnd_lower=common_rnd_lower
eat_rnd_upper=common_rnd_upper
eat_increment=common_increment
eat_decrement=common_decrement
eat_registered=('a_eat','a_move_to_food','a_pick_food')
eat_move_distance=common_move_distance

############### Grooming ##################
gr_lower_threshold=common_lower
gr_upper_threshold=common_upper
gr_latch_threshold=common_latch_threshold
gr_rnd_lower=common_rnd_lower
gr_rnd_upper=common_rnd_upper
gr_increment=common_increment
gr_decrement=common_decrement
gr_registered=('a_groom_with_target','a_move_to_groom_target','a_pick_groom_target')
gr_move_distance=common_move_distance

############### Exploring #################
ex_wander_steps=common_move_distance
ex_registered=('a_move_around',)

############### Resources #################
res_max_load=50
res_decrement=2
res_increment=1
res_delay=1

################# Dying ###################
die_registered=('a_stay_dead',)