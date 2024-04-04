## GROUP 24
## Group Members
| Surname   | Name                 |
| ----------| ---------------------|
| Osti      | Simone               |
| Russo     | Christian Francesco	 |
| Spinato   | Matteo               |

## Execution Instructions
**First way:** in a terminal execute the following ROS commands:
* `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library`
* `roslaunch tiago_iaslab_simulation navigation.launch`
* `rosrun assignment1 main_action_client_node _x:=11.0 _y:=0.0 _theta:=-90`
* `rosrun assignment1 main_action_server_node _CL_flag:=true`

**Second (alternative) way:** in a terminal execute the following ROS commands:
* `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library`
* `roslaunch tiago_iaslab_simulation navigation.launch`
* `roslaunch assignment1 tiago_navigation.launch`
