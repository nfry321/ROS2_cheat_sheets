---
description: Testing streaming  to laptop
---

# Picam

## Video4Linux

Quick google & check of ros index reveals 3 possible packages:

[https://github.com/Misterblue/ros2\_raspicam\_node](https://github.com/Misterblue/ros2_raspicam_node)

[https://github.com/christianrauch/raspicam2\_node](https://github.com/christianrauch/raspicam2_node)

[https://gitlab.com/boldhearts/ros2\_v4l2\_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera)

The 3rd link was found in an [issue](https://github.com/christianrauch/raspicam2_node/issues/11#issuecomment-699931002) in the 2nd link and is both more recently updated and is using a more generic method so maybe useful for other cameras too. Therefore trying that one first.

### V4l install

It is using video4linux, so first install this

```text
sudo apt-get update
sudo apt-get install v4l-utils
```

Some info on using v4l here:

{% embed url="https://www.techytalk.info/webcam-settings-control-ubuntu-fedora-linux-operating-system-cli/" %}

The driver creates several video devices [listed here](https://www.raspberrypi.org/documentation/usage/camera/raspicam/v4l2.md). You can see the connected devices and info on a specific device with -d &lt;number&gt;:

```text
v4l2-ctl --list-devices
v4l2-ctl --info -d 14
v4l2-ctl --list-formats -d 14
```

### ros2\_v4l2\_camera install

Install via [Github clone](../ros2-basics-1/installing-packages.md#git-repos) , run command to update dependencies, this will get image-transport.

Clone the dependancies for compressed transport [https://gitlab.com/boldhearts/ros2\_v4l2\_camera/-/tree/foxy/\#compressed-transport](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/foxy/#compressed-transport)

 Install the packages needed to build [https://gitlab.com/boldhearts/ros2\_v4l2\_camera/-/tree/foxy/\#building-ubuntu](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/foxy/#building-ubuntu)

Then `colcon build`.

To run the node use this command, with the device parameter set to 14\(?\) for a picam.

```text
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video14"
```

Currently \(04/11/20\) an issue with image size parameter using picam, but this has been raised and there is a PR in review that should close this [https://gitlab.com/boldhearts/ros2\_v4l2\_camera/-/issues/22](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/issues/22).



## Image-tools

In this tutorial [https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/](https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/) they use ros2 image-tool package. This worked on my laptop webcam \(and I dont have openCV installed I dont think\):

```text
# in 1st terminal
ros2 run image_tools showimage
# in 2nd terminal
ros2 run image_tools cam2image
```

But got an error on the pi so;

installed OpenCV - I used the require packages commands from [here](https://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation), then just used 

```text
$ sudo apt install libopencv-dev python3-opencv
```

Still could not open video stream, so openCV not the problem.



## other options

{% embed url="https://github.com/Kapernikov/cv\_camera" %}

{% embed url="https://github.com/ros-drivers/usb\_cam/tree/ros2" %}



