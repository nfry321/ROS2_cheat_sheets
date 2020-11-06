---
description: Tested micro-ros on Teensy 3.2
---

# Micro-Ros for Arduino IDE

{% embed url="https://discourse.ros.org/t/micro-ros-on-arduino/17009" caption="Discourse thread" %}

{% embed url="https://github.com/micro-ROS/micro\_ros\_arduino" caption="Github repo" %}

### Prerequisites 

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) if required. [Recommended](https://askubuntu.com/questions/107619/how-to-install-the-latest-arduino-ide) to unpack then move the folder to /opt [https://i.stack.imgur.com/7S9KB.png](https://i.stack.imgur.com/7S9KB.png)
2. Install [Teensyduino](https://www.pjrc.com/teensy/td_download.html) if required.
3. [Install](https://micro-ros.github.io/docs/tutorials/core/first_application_linux/) the micro\_ros\_setup package into a ROS2 workspace

### micro-ros for Arduino install

follow instructions in the [github readme](https://github.com/micro-ROS/micro_ros_arduino#how-to-use-the-precompiled-library) - 

1. Download pre-compiled library \(.zip\)
2. Include it in your project using `Sketch -> Include library -> Add .ZIP Library...`
3. [Patch Teensyduino](https://github.com/micro-ROS/micro_ros_arduino#patch-teensyduino)
4. Try example sketches - they were listed under incompatible in my examples menu but did upload.

### Usage

1. upload publisher example to teensy
2. connect via an agent
3. echo topic

### micro-ros agent

You need an agent that communicates with the micro-ros node & the rest of ROS2

In the [micro-ros setup repo ](https://github.com/micro-ROS/micro_ros_setup#building-micro-ros-agent)& in the [tutorial](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/freertos/)s it refers to building a micro-ros agent.

```text
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent serial --dev [device] -v6
```

Replace `[device]` with the result of `ls /dev/serial/by-id/*` this means even if the device number changes when disconnecting it will find the device via the name.

`-v6` verbose mode shows it communicating.

```text
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_6922840-if00 -v6
```

{% hint style="danger" %}
If you have ROS\_DOMAIN\_ID set then it will not show the topic!

check `echo ${ROS_DOMAIN_ID}` and if needed comment out of your .bashrc & reopen terminal
{% endhint %}

Check this issue for more details on how a domain ID can be configured if required: 

{% embed url="https://github.com/micro-ROS/micro\_ros\_arduino/issues/21" %}



### docker agent

The [readme](https://github.com/micro-ROS/micro_ros_arduino#how-to-use-the-precompiled-library) gives an example of an agent using docker. I got this to connect to the teensy, but unsure how/if it can connect to the rest of ROS2.

1. If required install docker `sudo snap install docker`
2. run this, may need to preface with sudo or follow these [instructions](https://docs.docker.com/engine/install/linux-postinstall/)  

```text
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:foxy serial --dev [YOUR BOARD PORT] -v6
```

I found [this](https://www.losant.com/blog/how-to-access-serial-devices-in-docker) helpful if you cant open the device. Replace `[your board port]` with `/dev/serial/by-id/<file_name_that_matches_your_device>` This means if the port number changes when unplugging it will still find the device. Use `ls /dev/serial/by-id/*` to find the name.  
This docker command connects and I can see that data is being exchanged. But now what? Can't see any topics or nodes listed. When I ctrl+c docker the Teensy goes to error loop \(led flashes\). - could be due to domain\_id issue above.

Presumably links to this tip in a [tutorial](https://micro-ros.github.io/docs/tutorials/core/first_application_linux/), but i don't want to go the docker route at the moment.

_**TIP:** Alternatively, you can use a docker container with a fresh ROS 2 Foxy installation. The one that serves the purpose is the container run by the command:_



