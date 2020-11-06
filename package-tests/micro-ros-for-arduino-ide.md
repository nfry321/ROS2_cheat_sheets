# Micro-Ros for Arduino IDE

{% embed url="https://discourse.ros.org/t/micro-ros-on-arduino/17009" caption="Discourse thread" %}

{% embed url="https://github.com/micro-ROS/micro\_ros\_arduino" caption="Github repo" %}

### Prerequisites 

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) if required. [Recommended](https://askubuntu.com/questions/107619/how-to-install-the-latest-arduino-ide) to unpack then move the folder to /opt [https://i.stack.imgur.com/7S9KB.png](https://i.stack.imgur.com/7S9KB.png)
2. Install [Teensyduino](https://www.pjrc.com/teensy/td_download.html) if required.

### micro-ros install

follow instructions in the github readme - 

1. Download and include the pre-compiled library \(.zip\)
2. Include it in your project using `Sketch -> Include library -> Add .ZIP Library...`
3. [Patch Teensyduino](https://github.com/micro-ROS/micro_ros_arduino#patch-teensyduino)
4. Try example sketches - they were listed under incompatible in my examples menu but did upload.

### Usage

1. upload publisher example to teensy
2. If required install docker `sudo snap install docker`
3. run this, may need to preface with sudo or follow these [instructions](https://docs.docker.com/engine/install/linux-postinstall/)  

```text
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:foxy serial --dev [YOUR BOARD PORT] -v6
```

I found [this](https://www.losant.com/blog/how-to-access-serial-devices-in-docker) helpful if you cant open the device. Replace `[your board port]` with `/dev/serial/by-id/<file_name_that_matches_your_devic`e`>` This means if the port number changes when unplugging it will still find the device.  


This docker command connects and I can see that data is being exchanged. But now what? Can't see any topics or nodes listed. When I ctrl+c docker the Teensy goes to error loop, so the connection is required.





