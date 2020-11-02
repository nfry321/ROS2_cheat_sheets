# Run on Multiple Machines

ROS2 does not need a ROS master so simply run the nodes and they will communicate over an ip network out of the box\(!\)

## Network Setup

### ROS\_IP

Hostnames need to resolve properly \(see for details: [http://wiki.ros.org/ROS/NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup)\). This may not happen so you can set the environmental variable  `export ROS_IP=ip.address.of.machine` \(do this every time or add to .bashrc but may change on different networks\). 

check it with: `echo $ROS_IP`

> My hostnames wouldn't resolve when ssh-ing but I didn't need to set the ROS\_IP for it to work.

### Domain ID

You need to have different domain id to keep separate from other ROS users on the network, this must be the same for all of your devices to connect. \([https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2\_setup/\#domain-id-allocation](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/#domain-id-allocation%20

%20)\)

Use the command below add to bashrc \(CHANGE NUMBER\)

```text
echo 'export ROS_DOMAIN_ID=20' >> ~/.bashrc
```

> If this is not set they should still communicate but good practice for working in the lab to avoid interference

## Test publish & subscribe

Test using these commands: \([https://discourse.ros.org/t/how-to-run-example-talker-listener-on-2-machines-with-different-ipaddr/2106/2](https://discourse.ros.org/t/how-to-run-example-talker-listener-on-2-machines-with-different-ipaddr/2106/2)\). Make sure you have done `source .bashrc` after changes.

```text
# on machine 1
ros2 run demo_nodes_cpp talker
# on machine 2
ros2 run demo_nodes_py listener
```

## Quality of Service \(QoS\) Settings

Quality of Service settings is a new addition to ROS2 because it uses DDS middleware.

{% embed url="https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/" %}

{% hint style="danger" %}
The pub and sub must have compatible QoS to talk
{% endhint %}

[https://surfertas.github.io/ros2/2019/08/17/ros2-qos.html](https://surfertas.github.io/ros2/2019/08/17/ros2-qos.html)

Requiring QoS is now default behaviour \(s[ince Dashing \)](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/#rclpy)

Can just use '10' for the depth as the was the previous behaviour, but there are also some default profiles that may be more appropriate \(for sensors for example\)

eg `self.pub = self.create_publisher(String, 'chatter', 10)`

 I found it hard to find the preset profiles, the descriptions are [here](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/#qos-profiles) but the actual 'code to type' I got from the [rclpy repo](https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/qos.py):

```text
 qos_profile_unknown
 qos_profile_system_default
 qos_profile_sensor_data
 qos_profile_services_default
 qos_profile_parameters
 qos_profile_parameter_events
 qos_profile_action_status_default
```

You have to import the definitions first eg:

```python
from rclpy.qos import qos_profile_sensor_data
```

 There is a [demo talker\_qos.py](https://github.com/ros2/demos/blob/foxy/demo_nodes_py/demo_nodes_py/topics/talker_qos.py) which is useful.

These examples go further \(i have not tried them out\):

{% embed url="https://github.com/ros2/demos/tree/master/quality\_of\_service\_demo" %}

## Network related links to check

If we get Wifi issues - Probably need to configure at DDS level? 

[https://discourse.ros.org/t/ros2-default-behavior-wifi/13460/9](https://discourse.ros.org/t/ros2-default-behavior-wifi/13460/9) 



