# Introduction to ROS (Robot Operating System)

- About ROS: http://www.ros.org/about-ros/

More information:
- ROS programming course at ETH, Zürich: http://www.rsl.ethz.ch/education-students/lectures/ros.html. Much of the info below is based on the slides from that course.

## What is ROS:

![](http://www.ros.org/wp-content/uploads/2013/12/ros_equation.png)

- Plumbing:
    - Process management
    - Inter-process communication
    - Device drivers
- Tools:
    - Simulation
    - Visualization
    - Graphical user interface
    - Data logging
- Capabilities:
    - Control
    - Planning
    - Perception
    - Mapping
    - Manipulation
- Eco system:
    - Package organization
    - Software distribution
    - Documentation 
    - Tutorials

## History:
- Developed in 2007 at the Stanford Artificial Intelligence Laboratory and the company Willow Garage
- Managed by the Open Source Robotic Foundation (OSRF) since 2013
- Used in products and by companies, research foundations and universities

## ROS Philosophy 

- Peer-to-peer
- Distributed
- Multilingual
- Light-weight
- Free and open-source

## ROS Programming

### ROS Master (http://wiki.ros.org/Master)

The ROS Master provides naming and registration services to the rest of the nodes in the ROS system. It tracks publishers and subscribers to topics as well as services. The role of the Master is to enable individual ROS nodes to locate one another. Once these nodes have located each other they communicate with each other peer-to-peer.

Start with: `roscore`


