Robot programming
======

This ros package was developed for the needs of the Robot Programming module CMP9767M at Lincoln University. The object of this project was to introduce the students to the environment of ros and ros packages as well as technics such as localisation, navigation, robot vision, coordination, handling tf transforms and mapping. The focus of this specific project was to create a rounded application that uses simple image processing technics to distinguish three different types of plants and detect harmful weeds near the plants. Moreover the navigation problem was solved using the ros topological navigation package. Finally, the simulation consists of two different robots, one robot identifies the bad weeds and the other robot scans the environment for the same harmfull weeds in order to spray them.

### Installation
Before installing the package or any dependencies do 
```sudo apt update && sudo apt upgrade```.
Every dependency will be inside the package.xml file. Go to the top directory of your catkin workspace where the source code of the ROS packages you'd like to use are. Then run:
```rosdep install --from-paths src --ignore-src -r -y```.
This command installs all the packages that the packages in your catkin workspace depend upon but are missing on your computer. 
If any package is missing do ```sudo apt install ros-<distro>-<missing_dep>```.
**Important**
This package depends on the package CMP9767M forked from LCAS/CMP9767M. It is **important** to also download this repo and include it inside your working workspace. [CMP9767M](https://github.com/parisChatz/CMP9767M.git)

### Executing
In order to run the complete simulation environment the following steps need to be followed.
1. Create a folder named mongodb in your home directory
2. `roslaunch uol_cmp9767m_base thorvald-sim.launch`
3. `roslaunch uol_cmp9767m_tutorial move_base_topo_nav.launch`
4. `roslaunch uol_cmp9767m_tutorial topo_nav.launch`
You will see some warnings in the terminal where you launched topo_nav.launch saying the pointset is not found in the message_store. This is because we haven't loaded the topological map to the mongodb yet. Once you do the next step, that warning should stop.
5. `rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/test.yaml`. *This step is required only once.*
6. `rviz -d $(rospack find uol_cmp9767m_tutorial)/config/topo_nav.rviz`
7. `roslaunch weed_detection_package my_nodes_launch.launch`
