---
description: TL;DR for keyboard teleop use teleop_twist_keyboard
---

# Teleoperation

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

~~Requires git clone install.~~  As of 05/11/20 these have been added to foxy [https://discourse.ros.org/t/new-packages-for-foxy-fitzroy-2020-11-05/17140](https://discourse.ros.org/t/new-packages-for-foxy-fitzroy-2020-11-05/17140)

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
2. [Teleop\_twist\_joy](https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/#foxy) - Good for simple driving: subs to joy and pubs twist messages \(robot velocity commands\)
3. [Joy teleop from Teleop tools](https://index.ros.org/p/joy_teleop/github-pal-robotics-joy_teleop/#foxy) - Good for being configurable to many robots and interfaces. 

### 1. Joy

> The joy package contains joy\_node, a node that interfaces a generic joystick to ROS 2. This node publishes a "Joy" message, which contains the current state of each one of the joystick's buttons and axes.

Check the readme for Topics & Parameters:

{% embed url="https://github.com/ros-drivers/joystick\_drivers/tree/ros2/joy" caption="Readme" %}

Installed by default, has 2 nodes `joy_enumerate_devices` and `joy_node`.

The first one lists the available devices and their ID, the 2nd is the node to connect to it, I didn't require any parameters beyond the defaults.

This may be useful if you have problems with the joystick being recognised, its for ROS1 but seems the same: [http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick)

There are tables of button index number for common joysticks in section 5 here: [http://wiki.ros.org/joy](http://wiki.ros.org/joy)

### 2. Teleop Twist Joy

There was an [issue](https://github.com/ros2/teleop_twist_joy/issues/17) with with repo not having any documentation so in the process of figuring out how to use it I have updated its [readme](https://github.com/ros2/teleop_twist_joy/blob/eloquent/README.md) and the changes have been merged - so read that!

The teleop\_node republishes Joy messages as scaled Twist messages.

So doesn't read the joystick itself \(uses `joy` for that\) but translates the raw buttons to a message type which is often used to describe \(differential drive\) robot motion \(eg used in Husky\).

Launching using the file provided by the package and by choosing a config file as an argument:

```text
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

{% hint style="warning" %}
This also starts the `joy` node so don't start one independently or there will be two with the same name
{% endhint %}

###  3. Teleop Tools joy\_teleop

{% embed url="https://github.com/ros-teleop/teleop\_tools/tree/foxy-devel/joy\_teleop" %}

{% embed url="http://wiki.ros.org/joy\_teleop" caption="Documentation \(ROS1\)" %}

This appears to be quite a widely applicable package, unlike the one above which is just for driving mobile bases, this can be used for humanoid robots. 

Looking at the example [config](https://github.com/ros-teleop/teleop_tools/blob/foxy-devel/joy_teleop/config/joy_teleop_example.yaml) it seems like this can be used to map joystick commands to topics, actions and services. 

To use setup the mappings you want in the config file then use a launch file to run it using that config.

```text
ros2 launch joy_teleop example.launch.py
```

It is simple enough to edit the config file to replicate the functionality of the teleop\_twist\_joy package.

```yaml
joy_teleop:
  ros__parameters:

    drive:
      type: topic
      interface_type: geometry_msgs/msg/Twist
      topic_name: cmd_vel
      deadman_buttons: [0] #A
      axis_mappings:
        linear-x:
          axis: 1
          scale: 0.5
          offset: 0
        angular-z:
          axis: 0
          scale: 0.5
          offset: 0
```

Note that the Joy node needs to be started separately as this subs to it.

Note that the deadman switch doesn't currently work but there is a fix that hasn't been merged \([https://github.com/ros-teleop/teleop\_tools/issues/45](https://github.com/ros-teleop/teleop_tools/issues/45)\)

