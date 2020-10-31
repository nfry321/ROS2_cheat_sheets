---
description: In progress
---

# Run on Multiple Machines

ROS2 does not need a ROS master so simply run the nodes and they will communicate over an ip network out of the box. 

## Network Setup

Hostnames need to resolve properly \(see for details: [http://wiki.ros.org/ROS/NetworkSetup](http://wiki.ros.org/ROS/NetworkSetup)\). This may not happen so you can set the environmental variable  `export ROS_IP=ip.address.of.machine` \(do this every time or add to .bashrc but may change on different networks\). 

check it with: `echo $ROS_IP`

You need to have different domain id to keep separate from other ROS users on the network, this must be the same for all of your devices to connect. \([https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2\_setup/\#domain-id-allocation](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/#domain-id-allocation%20

%20)\)

Use the command below add to bashrc \(CHANGE NUMBER\)

```text
echo 'export ROS_DOMAIN_ID=20' >> ~/.bashrc
```

## Test publish & subscribe

Test using these commands: \([https://discourse.ros.org/t/how-to-run-example-talker-listener-on-2-machines-with-different-ipaddr/2106/2](https://discourse.ros.org/t/how-to-run-example-talker-listener-on-2-machines-with-different-ipaddr/2106/2)\). Make sure you have done `source .bashrc` after changes.

```text
# on machine 1
ros2 run demo_nodes_cpp talker
# on machine 2
ros2 run demo_nodes_py listener
```

## Network related links to check

Quality of service rules is an advantage of ROS2 so needs understanding and using [  
  
 ](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_setup/#domain-id-allocation%20

%20)[https://github.com/ros2/demos/tree/master/quality\_of\_service\_demo](https://github.com/ros2/demos/tree/master/quality_of_service_demo) 

If we get Wifi issues - Probably need to configure at DDS level? 

[https://discourse.ros.org/t/ros2-default-behavior-wifi/13460/9](https://discourse.ros.org/t/ros2-default-behavior-wifi/13460/9) 



