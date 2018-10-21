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

Start with: 
```bash 
roscore
```

### ROS Nodes (http://wiki.ros.org/Nodes)

A node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming topics, RPC services, and the Parameter Server. These nodes are meant to operate at a fine-grained scale; a robot control system will usually comprise many nodes. For example, one node controls a laser range-finder, one Node controls the robot's wheel motors, one node performs localization, one node performs path planning, one node provide a graphical view of the system, and so on.

- Single purpose, executable program
- Individually compiled, executed, and managed
- Organized in *packages*.

Start a node:
```bash 
rosrun package_name node_name
```
List active nodes:
```bash
rosnode list
```

Get information about a node:
```bash
rosnode info node_name
```

### ROS Topics (http://wiki.ros.org/Topics)

Topics are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which decouples the production of information from its consumption. In general, nodes are not aware of who they are communicating with. Instead, nodes that are interested in data subscribe to the relevant topic; nodes that generate data publish to the relevant topic. There can be multiple publishers and subscribers to a topic.

- Nodes communicate over topics
    - Nodes can *publish* or *subscribe* to a topic
    - Typically, 1 publisher and *n* subscribers
- Topic is a name for a stream of messages

List active topics:
```bash 
rostopic list
```

Subscribe and print the content of a topic:
```bash
rostopic echo /topic
```

Get information aboout a node:
```bash
rostopic info /topic
```

### ROS Messages (http://wiki.ros.org/Messages)


Nodes communicate with each other by publishing messages to topics. A message is a simple data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types. Messages can include arbitrarily nested structures and arrays (much like C structs).

- Data structure defining the *type* of a topic
- Comprised of nested structure of integers, strings, etc, and arrays of objects
- Defined in *.msg files

See the type of a topic:
```bash
rostopic type /topic
```

Publish a message to a topic:
```bash
rostopic pub /topic type args
```

#### Examples:

`sensor_msgs/Image.msg`:

```bash
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

`geometry_msgs/Point.msg`:
```bash
float64 x
float64 y
float64 z
```

`geometry_msgs/PoseStamped`:
```bash 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Pose pose
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```


