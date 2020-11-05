---
description: TL;DR for keyboard teleop use teleop_twist_keyboard
---

# Teleop

## Keyboards

### Teleop Twist Keyboard

{% embed url="https://github.com/ros2/teleop\_twist\_keyboard" %}

This was forked from the[ teleop\_tools package](https://index.ros.org/r/teleop_tools/) and is the in ROS2 repo but doesn’t have foxy branch and less recently updated  

It has a release so can installed via apt-get install ros-foxy-teleop-twis-keyboard 

To Run  `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

This sends a twist message.

Has more keys mapped than other option, inc speed control. 

### Teleop Tools key\_teleop

{% embed url="https://index.ros.org/r/teleop\_tools/" %}

This has a recently updated foxy branch.

Requires git clone install. 

Includes keyboard, mouse and joystick options.

The mouse one works but the keyboard one gives error as no qos setting used. Easy fix and I submitted a pull request to fix it.

```text
ros2 run key_teleop key_teleop
ros2 run mouse_teleop mouse_teleop
```

But...It only uses the arrow keys and no speed control so the other package is better on keys. Mouse maybe useful but feels a bit harder to precisely control. 

## Joysticks

Searching ros index reveals 3 relevant foxy packages

1. [joy](https://index.ros.org/p/joy/github-ros-drivers-joystick_drivers/#foxy) - Driver to interface with joystick and output joy messages
2. [Teleop\_twist\_joy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/#foxy) - subs to joy and pubs twist messages \(robot velocity commands\)
3. [Joy teleop from Teleop tools](https://index.ros.org/p/joy_teleop/github-pal-robotics-joy_teleop/#foxy)

### 1. Joy

> The joy package contains joy\_node, a node that interfaces a generic joystick to ROS 2. This node publishes a "Joy" message, which contains the current state of each one of the joystick's buttons and axes.

Check the readme for Topics & Parameters:

{% embed url="https://github.com/ros-drivers/joystick\_drivers/tree/ros2/joy" caption="Readme" %}

Installed by default, has 2 nodes `joy_enumerate_devices` and `joy_node`.

The first one lists the available devices and their ID, the 2nd is the node to connect to it, I didn't require any parameters beyond the defaults.

This may be useful if you have problems with the joystick being recognised, its for ROS1 but seems the same: [http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

### 2. Teleop Twist Joy

The teleop\_node republishes Joy messages as scaled [geometry\_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) messages.

So doesn't read the joystick itself \(use _joy_ for that\) but translates the raw buttons to a message type which is often used to describe robot motion so plug into a further controller node \(eg used in Husky\).

This has an xbox config file, but need to work out how to run it using that!

No ROS2 Documentation, but is for ROS1 which should be similar: [http://wiki.ros.org/teleop\_twist\_joy](http://wiki.ros.org/teleop_twist_joy)

```text
ros2 run teleop_twist_joy teleop_node 
```

###  3. Teleop Tools joy\_teleop

```text
ros2 run joy_teleop joy_teleop
```

### 

