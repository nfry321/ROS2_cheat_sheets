---
description: Seems sensible
---

# Diagnostics

When looking into sending status info back from a micro-ros node I started to look into the diagnostic messages, and it turns out there are lots of tools for using these. As it seems like something we want to build in from the start I'm recording what I learn here. 

[Wiki with documentation](http://wiki.ros.org/diagnostics). There is a [ROS2 foxy ](https://github.com/ros/diagnostics/tree/foxy)branch on the repo. 

### Micro-ros

Obviously on an embedded device we can't run these packages but we can publish in the correct format for it to integrate with the existing tools.

#### Existing message definitions:

* Whole robot status as array of DiagnosticStatus messages; [DiagnosticArray](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html)
* Single status message from a individual component of the robot; [DiagnosticStatus](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticStatus.html) 
* Single parameter from the individual component. Can have an array of these within your DiagnosticStatus; [KeyValue](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/KeyValue.html)

Creating and filling in a message isn't super simple in micro-ros if it's not a basic type.



