# Mobile Robot Simulation using ROS, Gazebo and Jupyter Notebook

## Research Track II - Assignment II
Muhammad Ali Haider Dar, _[5046263@studenti.unige.it](mailto:5046263@studenti.unige.it)_

MSc Robotics Engineering, University of Genoa, Italy

Instructor: [Prof. Carmine Recchiuto](https://rubrica.unige.it/personale/UkNDWV1r)

## Installation

The repository provided includes packages and a simulation environment for controlling a mobile robot in the Gazebo simulator. The main focus of this project is to improve the user interface, which has been developed in Jupyter Notebook. This assignment serves as an extension of its previous iteration, assignment 1, which can be accessed at: https://github.com/alihaidersays/research-track-2-a1.

Before proceeding with the instructions, it is important to have ROS1 installed on your system. Please ensure that you have it installed before proceeding. The repository consists of three branches. The first branch is named "main", and the second branch is named "sphinx". Both branches contain the same project but with different styles of documentation. The "main" branch utilizes Doxygen for documentation, while the "sphinx" branch utilizes the Sphinx tool.

To successfully deploy the code available in the "main" branch, follow the steps below:

1. Install the code in the ROS workspace at {ros_ws}/src and run the following commands to build the workspace:
    * Run the following command to build the workspace:
      ```
      catkin_make
      ```
    * Navigate to the `devel/` directory:
      ```
      cd devel/
      ```
    * Source the `setup.bash` file:
      ```
      source setup.bash
      ```
2. Ensure that Jupyter Notebook is installed on your system. If not, run the following commands to install it:
```
pip3 install jupyter bqplot pyyaml ipywidgets
jupyter nbextension enable --py widgetsnbextension
```
3. Run the following command to check if Jupyter Notebook is working correctly:
```
jupyter notebook --allow-root
```
4. If your Firefox browser is not up-to-date, run the following command to upgrade it:
```
apt-get install firefox
```
5. If you intend to use this project in a Docker container, you can run Jupyter Notebook within the container by exposing the port to the host computer. Start a new container and forward port 8888, then run the following command:
```
jupyter notebook --allow-root --ip 0.0.0.0
```
6. After running the above command in the Docker container, open any browser on your host system and go to the following address:
```
http://localhost:8888/
```
7. In the Jupyter Notebook hub on the browser, navigate to the directory `{ros_ws}/src/rt2_assignment1/` and open the `scripts.ipynb` file. This notebook contains the user interface of the project. However, do not interact with the script yet, as the simulation environment needs to be set up first.
8. The "sourcefile" branch contains the required source files for running the project. Clone the branch and install the files in the `/root` directory. Make the source file executable by running the following command:
```
chmod +x <file_name>
```
9. If Gazebo simulation environments are not already installed on your system, please follow the provided instructions [here](http://gazebosim.org/tutorials?tut=install_ubuntu) to install them.

## Gazebo Simulation using Action Server

The Gazebo simulation using the Action Server consists of four main nodes:

1. `position_service`: This node provides a service for retrieving the position of the robot in the simulation. It subscribes to the odom topic and responds to service requests with the current position information.
2. `state_machine_action`: This node manages the state transitions of the robot. It utilizes an action server to assign random target positions to the robot within the simulation. Upon reaching a target, a new random target is assigned. This process continues until interrupted.
3. `go_to_point_action.py`: This node implements the action server for controlling the robot's movements. It receives target positions from the state_machine_action node and controls the robot's motion to reach the specified targets.
4. `user_interface_action.py`: This node provides the user interface for interacting with the simulation. It utilizes Jupyter Notebook and allows users to control the robot's movements, adjust velocities, and visualize feedback data.

To run this part of the project, ensure that you are in the `/root` folder where the `rt2_assignment2.sh` bash file is located. Open a terminal and execute the following command:
```
./rt2_assignment2.sh
```
After running the command, two terminal windows will appear on the screen, including the Gazebo simulation with a mobile robot. Wait for the system to load all the necessary files. You are now ready to interact with the Jupyter Notebook script and explore the functionality of the simulation.

## Documentation
 
### Doxygen
The documentation for Doxygen can be accessed here: `{ros_ws}/src/rt2_assignment2/docs/html/index.html`

### Sphinx 
The documentation for Sphinx can be accessed here:  `{ros_ws}/src/rt2_assignment2/_build/html/index.html`
