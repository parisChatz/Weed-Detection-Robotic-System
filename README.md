# Weed Control Robotic System

This ros package was developed for the needs of the Robot Programming module CMP9767M at Lincoln University. The object of this project was to introduce the students to the environment of ros and ros packages as well as technics such as localisation, navigation, robot vision, coordination, handling tf transforms and mapping. The focus of this specific project was to create a rounded application that uses simple image processing technics to distinguish three different types of plants and detect harmful weeds near the plants. After the detection is complete then another agent is deployed to precisely spray the detected harmful weeds.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

First create a ROS worspace (e.g. home/catkin_ws) and a src folder inside the workspace. Put the whole repository in the catkin_ws/src folder. Finaly, complile the catkin_ws.

### Prerequisites

Ubuntu 16.04 and ROS kinetic were used for this project. 

### Installing

A step by step series of examples that tell you how to get a development environment running.

First of all, it is assumed that ROS kinetic is correctly installed. If not please follow the [instructions](http://wiki.ros.org/ROS/Installation) for installing ROS kinetic.

1. Open a new terminal and do ```sudo apt update && sudo apt upgrade```.
2. Create a new catkin workspace, navigate inside the workspace, create a src folder and clone the repository inside the src folder (e.g.`mkdir catkin_ws && cd catkin_ws && mkdir src && cd src && git clone https://github.com/parisChatz/Weed-Detection-Robotic-System.git`)
3. Navigate to the base of the catkin_ws (e.g. catkin_ws)
4. In the terminal do `catkin_make`.
5. Every dependency will be inside the package.xml file. Go to the top directory of your catkin workspace (e.g. `cd catkin_ws`). Then run: ```rosdep install --from-paths src --ignore-src -r -y```.

Finally, open a new terminal, source the workspace `source catkin_ws/devel/setup.bash` and run `roslaunch uol_cmp9767m_base thorvald-sim.launch` to make sure that everything is up and running. This should run gazebo with 2 stationary robots.

If any package is missing do ```sudo apt install ros-<distro>-<missing_dep>```.

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


## Built With

* [Robot Operating System ](https://www.ros.org/)


## Authors

 **Paraskevas Chatzithanos**  - [GitHub](https://github.com/parisChatz)


## License

This project is licensed under the "blank" License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

This package depends on the package [CMP9767M](https://github.com/parisChatz/CMP9767M.git) forked from LCAS/CMP9767M.