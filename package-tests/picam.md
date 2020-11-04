---
description: Testing streaming  to laptop
---

# Picam

Quick google & check of ros index reveals 3 possible packages:

[https://github.com/Misterblue/ros2\_raspicam\_node](https://github.com/Misterblue/ros2_raspicam_node)

[https://github.com/christianrauch/raspicam2\_node](https://github.com/christianrauch/raspicam2_node)

[https://gitlab.com/boldhearts/ros2\_v4l2\_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera)

The 3rd link was found in an [issue](https://github.com/christianrauch/raspicam2_node/issues/11#issuecomment-699931002) in the 2nd link and is both more recently updated and is using a more generic method so maybe useful for other cameras too. Therefore trying that one first.

It is using video4linux, so first install this

```text
sudo apt-get update
sudo apt-get install v4l-utils
```

Some info on using v4l here:

{% embed url="https://www.techytalk.info/webcam-settings-control-ubuntu-fedora-linux-operating-system-cli/" %}

Try the first part of this first [https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/](https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/)



