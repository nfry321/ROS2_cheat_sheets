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
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent serial --dev [device] -v6
```

{% hint style="warning" %}
On Pi3 build agent takes a verrry long timer \(does it even complete?\)

On Pi4 build agent took 20 mins.
{% endhint %}

Replace `[device]` with the result of `ls /dev/serial/by-id/*` this means even if the device number changes when disconnecting it will find the device via the name.

`-v6` verbose mode shows it communicating.

```text
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_6922840-if00 -v6
```

{% hint style="warning" %}
Your ROS\_DOMAIN\_ID must either be clear or match that set in the micro-ros node.

check `echo ${ROS_DOMAIN_ID}`
{% endhint %}

After [raising an issue](https://github.com/micro-ROS/micro_ros_arduino/issues/21#issuecomment-724558682) the inability to set the domain ID was fixed \(main author is very responsive so don't hesitate to reach out if needed!\). In version 0.0.2 now you can change the Domain ID of your nodes using something like:

```text
...
rcl_node_options_t node_ops = rcl_node_get_default_options();
node_ops.domain_id = 10;
RCCHECK(rclc_node_init_with_options(&node, "my_node_name", "", &support, &node_ops));
...
```



### docker agentlinux

The [readme](https://github.com/micro-ROS/micro_ros_arduino#how-to-use-the-precompiled-library) gives an example of an agent using docker. I got this to connect to the teensy, ~~but unsure how/if it can connect to the rest of ROS2.~~ Probably fixed by the domain id issue, but not tried.

1. If required install docker `sudo snap install docker`
2. run this, may need to preface with sudo or follow these [instructions](https://docs.docker.com/engine/install/linux-postinstall/)  

```text
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:foxy serial --dev [YOUR BOARD PORT] -v6
```

I found [this](https://www.losant.com/blog/how-to-access-serial-devices-in-docker) helpful if you cant open the device. Replace `[your board port]` with `/dev/serial/by-id/<file_name_that_matches_your_device>` This means if the port number changes when unplugging it will still find the device. Use `ls /dev/serial/by-id/*` to find the name.  
This docker command connects and I can see that data is being exchanged. But now what? Can't see any topics or nodes listed. When I ctrl+c docker the Teensy goes to error loop \(led flashes\). **- could be due to domain\_id issue above.**

Presumably links to this tip in a [tutorial](https://micro-ros.github.io/docs/tutorials/core/first_application_linux/), but i don't want to go the docker route at the moment.

_**TIP:** Alternatively, you can use a docker container with a fresh ROS 2 Foxy installation. The one that serves the purpose is the container run by the command:_

### Message creation

[https://micro-ros.github.io/docs/tutorials/core/programming\_rcl\_rclc/](https://micro-ros.github.io/docs/tutorials/core/programming_rcl_rclc/)

The string `Hello World!` can be assigned directly to the message of the publisher `pub_msg.data`. First the publisher message is initialized with `std_msgs__msg__String__init`. Then you need to allocate memory for `pub_msg.data.data`, set the maximum capacity `pub_msg.data.capacity` and set the length of the message `pub_msg.data.size` accordingly. You can assign the content of the message with `snprintf` of `pub_msg.data.data`.

```text
  // assign message to publisher
  std_msgs__msg__String__init(&pub_msg);
  const unsigned int PUB_MSG_CAPACITY = 20;
  pub_msg.data.data = malloc(PUB_MSG_CAPACITY);
  pub_msg.data.capacity = PUB_MSG_CAPACITY;
  snprintf(pub_msg.data.data, pub_msg.data.capacity, "Hello World!");
  pub_msg.data.size = strlen(pub_msg.data.data);
```

### 

### Not Enough memory in buffer issue.

When sending longer values you may get an error. Check this [issue](https://github.com/micro-ROS/micro_ros_arduino/issues/23) - you need to initialise your message properly. and update the size when you use it.

```cpp
// Init
std_msgs__msg__String msg;
msg.data.data = (char*)malloc(200*sizeof(char));
msg.data.size = 0;
msg.data.capacity = 200;

// Use
char my_str[] = "TEST";
memcpy(msg.data.data, my_str, strlen(my_str));
msg.data.size = strlen(msg.data.data);
rcl_publish(&publisher, &msg, NULL);
```

### Custom Messages

{% embed url="https://github.com/micro-ROS/micro\_ros\_arduino/issues/14\#issuecomment-722242175" caption="Use these instructions" %}

First clone the repo.

As we have a repo of our custom messages, go to `extras/library_generation/extra_packages` and edit the `extra_packages.repos` to point to that that.

```text
repositories:
  control_msgs:
    type: git
    url: https://github.com/ros-controls/control_msgs
    version: foxy-devel
  pipebot_msgs:
    type: git
    url: https://github.com/pipebots/pipebot-msgs.git
    version: nfry321-sprintbot-leds
```

build:

```text
# Go to this library folder installation
cd ~/Arduino/libraries/micro_ros_arduino-0.0.1

# Use the docker to build all the necessary stuff:
docker pull microros/micro_ros_arduino_builder:latest
# build just for Teensy 3
docker run -it --rm -v $(pwd):/arduino_project microros/micro_ros_arduino_builder:latest -p teensy3
```

You should then find your custom msg named in `available_ros2_types`

To get this into the arduino IDE; Zip the folder and add in using Sketch &gt; Include Library &gt; Add .zip library. You will need to delete the existing one from your Arduino/libraries folder. 

