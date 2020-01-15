# ROS Repository for the MIRANA Project

This repository provides all required packages for autonomous navigation and mapping the MIRANA robot, a mobile robot based on the Segway Loomo.

### Requirements

- Ubuntu 16.04, ROS Kinetic (tested) or Ubuntu 18.04, ROS Melodic (not tested)

### Usage

- Start by open a new terminal and start a roscore

  ```bash
  roscore
  ```

- Make sure that the MIRANA robot is connected with the same network as the used computer. Start the Android application on the robot (for further details we refer to the GitHub repo for the Android application). Launch the autonomous mapping procedure

  ```bash
  roslaunch automap automap.launch
  ```

- After the mapping procedure has finished, the map is stored. The MIRANA robot can now use this map for navigation starting

  ```
  roslaunch automap autonavigation.launch
  ```

  

