---
description: TL;DR for keyboard teleop use first pkg
---

# Teleop

## Teleop Twist Keyboard

{% embed url="https://github.com/ros2/teleop\_twist\_keyboard" %}

This was forked from the[ teleop\_tools package](https://index.ros.org/r/teleop_tools/) and is the in ROS2 repo but doesn’t have foxy branch and less recently updated  

It has a release so can installed via apt-get install ros-foxy-teleop-twis-keyboard 

To Run  `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

This sends a twist message.

Has more keys mapped than other option, inc speed control. 

There is also [https://github.com/ros2/teleop\_twist\_joy](https://github.com/ros2/teleop_twist_joy) which may or may not be similarly working/good 

This may be useful when looking into joysticks: [http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

## Teleop Tools

{% embed url="https://index.ros.org/r/teleop\_tools/" %}

This has a recently updated foxy branch.

Requires git clone install. 

Includes keyboard, mouse and joystick options.

The mouse one works but the keyboard one gives error as no qos setting used. Easy fix and I submitted a pull request to fit it.

But...It only uses the arrow keys and no speed control so the other package is better on jeys. Mouse maybe useful but feels a bit harder to precisely control. 

