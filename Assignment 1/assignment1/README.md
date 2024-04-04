GROUP 24
Matteo Spinato, matteo.spinato@studenti.unipd.it
Christian Francesco Russo, christianfrancesco.russo@studenti.unipd.it
Simone Osti, simone.osti@studenti.unipd.it

Commands to run the code:
 - roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
 - roslaunch tiago_iaslab_simulation navigation.launch
 - rosrun assignment1 main_action_client_node _x:=11.0 _y:=0.0 _theta:=-90
 - rosrun assignment1 main_action_server_node _CL_flag:=true
 
Or, you can use the launch file instead of the last two commands:
 - roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
 - roslaunch tiago_iaslab_simulation navigation.launch
 - roslaunch assignment1 tiago_navigation.launch

