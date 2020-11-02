---
description: In progress
---

# Using ROS1 Packages

Being a relatively early adopter of ROS means that most packages still only work in ROS1. Here I'm exploring how to use them. There are two options;

1. Migrate whole package to ROS2, often as a branch in the same repository. Intention is that most of the underlying code remains the same. [https://index.ros.org/doc/ros2/Contributing/Migration-Guide/](https://index.ros.org/doc/ros2/Contributing/Migration-Guide/)
2. Use the ROS package and use a bridge to communicate with ROS2

## 1. Migration

Clear guide of how to actually go about it without breaking functionality and testing node by node. No technical details though.

{% embed url="https://roboticsbackend.com/migrate-ros-project-from-ros1-to-ros2/" %}

Official Guide:

{% embed url="https://index.ros.org/doc/ros2/Contributing/Migration-Guide/" %}

There are some tools to do this, notably from Amazon, which may do much of the heavy lifting automatically?

{% embed url="https://index.ros.org/doc/ros2/Contributing/Examples-and-Tools-for-ROS1----ROS2-Migrations/" %}

Tutorial to look at:

{% embed url="https://industrial-training-master.readthedocs.io/en/melodic/\_source/session7/ROS1-to-ROS2-porting.html" %}



## 2. Bridge

{% embed url="https://github.com/ros2/ros1\_bridge/tree/foxy" %}

Tutorial to look at:

{% embed url="https://www.allisonthackston.com/articles/bridging\_ros\_ros2.html" %}



## 3. SOSS

Might be a 3rd option, needs looking into

{% embed url="https://discourse.ros.org/t/soss-a-whole-new-approach-to-your-ros-1-ros-2-bridge/17040" %}

{% embed url="https://soss.docs.eprosima.com/en/latest/getting\_started.html" %}



