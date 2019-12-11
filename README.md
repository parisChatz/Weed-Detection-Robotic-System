Robot programming
======

##### This ros package was developed for the needs of the Robot Programming module CMP9767M at Lincoln University. The object of this project was to introduce the students to the environment of ros and ros packages as well as technics such as localisation, navigation, robot vision, coordination, handling tf transforms and mapping. The focus of this specific project was to create a rounded application that uses simple image processing technics to distinguish three different types of plants and detect harmful weeds near the plants. Moreover the navigation problem was solved using the ros topological navigation package. Finally, the simulation consists of two different robots, one robot identifies the bad weeds and the other robot scans the environment for the same harmfull weeds in order to spray them.

### Installation
Before installing the package or any dependencies do `sudo apt update && sudo apt upgrade
`.Every dependency will be inside the package.xml file. Go to the top directory of your catkin workspace where the source code of the ROS packages you'd like to use are. Then run:`rosdep install --from-paths src --ignore-src -r -y`.This command installs all the packages that the packages in your catkin workspace depend upon but are missing on your computer. 
If any package is missing do `sudo apt install ros-<distro>-<missing_dep>`.

