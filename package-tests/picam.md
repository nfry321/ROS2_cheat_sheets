---
description: Testing streaming from Rpi4 with Ubuntu Server 20.04 to laptop
---

# Pi camera

## Enable Pi camera on Ubuntu 20.04

Before using the camera it must be enabled. On this OS we do not have raspi-config which would be the usual way to do it in raspbian.

You need to edit this file using: `sudo nano /boot/firmware/config.txt`

Then add `start_x=1` at the end. Save and reboot. 

## ROS2 Package Options

Quick google & check of ros index reveals 3 possible packages:

[https://github.com/Misterblue/ros2\_raspicam\_node](https://github.com/Misterblue/ros2_raspicam_node)

[https://github.com/christianrauch/raspicam2\_node](https://github.com/christianrauch/raspicam2_node)

[https://gitlab.com/boldhearts/ros2\_v4l2\_camera](https://gitlab.com/boldhearts/ros2_v4l2_camera)

The 3rd link was found in an [issue](https://github.com/christianrauch/raspicam2_node/issues/11#issuecomment-699931002) in the 2nd link and is both more recently updated and is using a more generic method so maybe useful for other cameras too. Therefore trying that one first.

## V4l install

It is using video4linux, so first install this

```text
sudo apt-get update
sudo apt-get install v4l-utils
```

Some info on using v4l here:

{% embed url="https://www.techytalk.info/webcam-settings-control-ubuntu-fedora-linux-operating-system-cli/" %}

The driver creates several video devices [listed here](https://www.raspberrypi.org/documentation/usage/camera/raspicam/v4l2.md). The one you need to use is `/dev/video0`. You can see the connected devices and info on a specific device with -d &lt;number&gt;:

```text
v4l2-ctl --list-devices
v4l2-ctl --info -d 0
v4l2-ctl --list-formats -d 0
```

## ROS2 V4L2\_camera package

{% embed url="https://index.ros.org/p/v4l2\_camera/gitlab-boldhearts-ros2\_v4l2\_camera/\#foxy" %}

There is a release so it, and the dependencies, can easily be installed with apt-get. \(as of 19/11/2020 this isn't the latest version with [the fix for pi cams ](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/issues/22.)so need to build from source\)

```text
sudo apt-get install ros-foxy-v4l2-camera
```

#### Build from source following the instructions from the developer in this article: 

{% embed url="https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304" %}

ROS 2 workspace in `~/ros2_foxy` sourced first:

```text
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git
$ git clone --branch ros2 https://github.com/ros-perception/vision_opencv.git
$ git clone --branch ros2 https://github.com/ros-perception/image_common.git
$ git clone --branch ros2 https://github.com/ros-perception/image_transport_plugins.git
$ cd ..
$ rosdep install --from-paths src -r -y
```

Now build everything we need and source the new workspace:

```text
$ colcon build --packages-up-to v4l2_camera image_transport_plugins
$ source install/local_setup.bash
```

And finally you can run the camera node:

```text
$ ros2 run v4l2_camera v4l2_camera_node
```

### Receiving side

View the camera output, for instance by running the RQT image viewer: 

```text
ros2 run rqt_image_view rqt_image_view
```

For me this ground to a halt very quickly with the raw data. To view the compressed images you need to install the plugins mentioned in the [readme](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/foxy#compressed-transport). 

```text
$ cd path/to/workspace
$ git clone https://github.com/ros-perception/vision_opencv.git --branch ros2 src/vision_opencv
$ git clone https://github.com/ros-perception/image_transport_plugins.git --branch ros2 src/image_transport_plugins
$ rosdep install --from-paths src -r -y
($ colcon build --packages-up-to vision_opencv) # I think this isnt requred and the next command will build this as well
$ colcon build --packages-up-to image_transport_plugins
```

### Republish for other packages

Some packages do not support the compressed images so they need to be republished uncompressed. Using[ readme instructions](https://gitlab.com/boldhearts/ros2_v4l2_camera/-/tree/foxy#usage).

```text
ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed
```

If you remap to `/image` topic then you can use `ros2 run image_tools showimage` to view.

```text
ros2 run image_transport republish compressed --ros-args --remap in/compressed:=image_raw/compressed raw --remap out:=image
```

### Camera Type

####  Raspberry Pi Camera Rev 1.3.

{% hint style="success" %}
Working. Tested all of the above on RPi4 & Ubuntu 20.04.
{% endhint %}

#### Arducam OV9281 1MP Monochrome Global Shutter Camera 

{% hint style="warning" %}
Publishing, haven't managed to view.
{% endhint %}

This suggests that [only Rev. B is compatible with the V4L2 driver ](https://www.arducam.com/product/ov9281-mipi-1mp-monochrome-global-shutter-camera-module-m12-mount-lens-raspberry-pi/)and I think I have an earlier board as Rev.B not printed on it, but it appears to be publishing images.

There are [reports](https://www.arducam.com/forums/topic/no-activity-from-ov9281-with-v4l2-ctl-on-rpi-4/) of it not working on Ubuntu. The[ MiPi drivers ](https://github.com/ArduCAM/MIPI_Camera/tree/master/RPI)which we used previously are also not compatible with Ubuntu.

#### Trying to follow the guide anyway...

{% embed url="https://www.arducam.com/docs/cameras-for-raspberry-pi/migrated/ov9281-1mp-global-shutter-raspberrypi-camera/" %}

1. The overlays are in a slightly different location : `ls /boot/firmware/overlays/ov9281.dtbo`
2. As is the config file: `sudo nano /boot/firmware/config.txt`
3. **Does output successfully on this command  and publish using cam2image**.`v4l2-ctl --stream-mmap --stream-count=-1 -d /dev/video0 --stream-to=/dev/null`

When trying to run the ros node it seems like the pixel and encoding formats are not supported, presumably because it is greyscale?

## Image\_tools

In this tutorial [https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/](https://index.ros.org/doc/ros2/Tutorials/Quality-of-Service/) they use ros2 image-tool package. This worked on my laptop webcam.

```text
# in 1st terminal (where viewing)
ros2 run image_tools showimage
# in 2nd terminal (or pi with camera attached)
ros2 run image_tools cam2image
```

On RPi with pi cam: Had to install the image\_tools package, so not included in basic ros install.

`sudo apt-get install ros-foxy-image-tools` 

This 'works', the images are published and I can subscribe and see them for a bit but showimage freezes very quickly. 

## Other Options

{% embed url="https://github.com/Kapernikov/cv\_camera" %}

{% embed url="https://github.com/ros-drivers/usb\_cam/tree/ros2" %}



