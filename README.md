# Mobile Robot Simulation using ROS, Gazebo and Jupyter Notebook

## Research Track II - Assignment II
Muhammad Ali Haider Dar, _[5046263@studenti.unige.it](mailto:5046263@studenti.unige.it)_

MSc Robotics Engineering, University of Genoa, Italy

Instructor: [Prof. Carmine Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

## Installation

The repository provided includes packages and a simulation environment for controlling a mobile robot in the Gazebo simulator. The main focus of this project is to improve the user interface, which has been developed in Jupyter Notebook. This assignment serves as an extension of its previous iteration, assignment 1, which can be accessed at: https://github.com/alihaidersays/research-track-2-a1.

Before proceeding with the instructions, it is important to have ROS1 installed on your system. Please ensure that you have it installed before proceeding. The repository consists of three branches. The first branch is named "main", and the second branch is named "sphinx". Both branches contain the same project but with different styles of documentation. The "main" branch utilizes Doxygen for documentation, while the "sphinx" branch utilizes the Sphinx tool.

1. Code available in **Main** should be install in ROS workspace {ros_ws}/src and to deploy successfully run the following command.
```
catkin_make
cd devel/
source setup.bash
```
2. Next dependency is to install the jupyter Notebook in your system. It k is not by default included with Python, so run the following commands to install Jupyter.
```
pip3 install jupyter bqplot pyyaml ipywidgets
jupyter nbextension enable --py widgetsnbextension
```
3. Now run the Jupyter Notebook to check if its working fine using following command. 
```
jupyter notebook --allow-root
```
4. If Firefox browser in your system is not upgraded then you can use the following command to do so.
```
apt-get install firefox
```
5. It might be the case that you want to use this project in a Docker and in that case Jupyter may be used in a Docker container. Using the port forwarding option the juyter server will be exposed to the host computer. However in this case you should start a brand new container, by also forwarding port 8888. Once that is done you can simply run the following command. 
```
jupyter notebook --allow-root --ip 0.0.0.0
```
6. After running this command in docker you can open any browser in your host system and go to the localhost server. 
```
http://localhost:8888/
```
7. In the jupyter notebook hub on the browser, you can navigate to this directory '{ros_ws}/src/rt2_assignment1/' and open the 'scripts.ipynb' jupyter notebook file. This file contains the user interface of the project, but don't interact with this script yet. We need to setup simulation environment first.

8. Code available in **sourcefile** branch is a source files that will be required to run the project. Please install them in your /root directory. Once the branch is cloned make this file executable by running the following command.
```
chmod +x <file_name>
```
9. This project requires Gazebo simulation environments, if they are not already installed in your system please install them by following given instructions: 

  **Gazebo:** 
  
  * Please follow the instructions available on the respository.http://gazebosim.org/tutorials?tut=install_ubuntu

## Gazebo Simulation using Action Server

The project is consist of four main nodes. 

1. position_service 
2. state_machine_action 
3. go_to_point_action.py
4. user_interface_action.py

In order to run this part please make sure you are in /root folder where you have already downloaded **rt2_assignment2.sh** bash file. Open the terminal and run the following command.

```
./rt2_assignment2.sh
```
After doing this you will see two terminal windows start appearing on the screen, including a Gazebo simulation with a mobile robot in it. Wait for the system load all the files. Now you are ready to interact with the Jupyter notebook script and we have already setup in setup 3. 

## Documentation
 
 **Doxygen**
 
 * The documentation for Doxygen can be access from this directory '{ros_ws}/src/rt2_assignment2/docs/html/index.html'
 
 **Sphinx** 
 
 * The documentation for Sphinx can be access from this directory '{ros_ws}/src/rt2_assignment2/_build/html/index.html'
