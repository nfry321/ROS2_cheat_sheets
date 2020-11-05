---
description: Terminal only install for robots using Ubuntu Server 20.04 and ROS2 Foxy
---

# Install on RPi

> Currently tested on Pi 3, should work on pi 4

## Install Ubuntu 20.04 Server

This way should mean you can configure wi-fi before first boot then ssh in using the static ip you set, so no monitor or keyboard is needed.

\(Based on this [https://raspberrypi.stackexchange.com/a/113642](https://raspberrypi.stackexchange.com/a/113642)\)

1. Image the SD Card using Raspberry Pi Imager v 1.3 selecting the 'UNBUNTU 20.04 LTS \(RASBERRY PI 3/4\)\(64-bit\) image.  \(or download from here [https://ubuntu.com/download/raspberry-pi](https://ubuntu.com/download/raspberry-pi) and use balena etcher\)
2. When imaging is completed, remove the SD Card and then reinsert it so that it is mounted. 
3. Modify 'network-config' found on the SD Card so that it contains only the items in the example below. Comment out with \#, or remove, all other settings including the LAN ones. Enter any missing items. Be certain to maintain only the indentations shown. Use two spaces for each indentation. Remove all tab characters. Be especially careful if using Notepad++ as it will enter tabs wherever enter is used to start a new line. Replace the SSID with your wireless SSID and the PassPhrase with your wireless passphrase. When done, those two values should be wrapped in quotes. Save the modified file to the SD Card. 

```text
# This file contains a netplan-compatible configuration which cloud-init
# will apply on first-boot. Please refer to the cloud-init documentation and
# the netplan reference for full details:
#
# https://cloudinit.readthedocs.io/
# https://netplan.io/reference
#

version: 2
renderer: networkd
wifis:
  wlan0:
    dhcp4: true
    dhcp6: true
    optional: true
    access-points:
      "SSID":
         password: "PassPhrase"
```

For a static ip add the details below but fill in your numbers.

Use `ip addr show` to find existing ip address. Look for entry like "inet 192.168.0.64/24 brd...."

Use `ip route show` to fill in gateway4. Look for something like "default via 192.168.0.1 dev...."

Use `systemd-resolve --status` and look for section that says "DNS servers" to fill in name servers" \(q to quit\)

```text
dchp4: false 
dchp6: false 
addresses: [ip to set/24] 
gateway4: 192.168.0.1 
nameservers: 
  addresses: [194.168.4.100, 194.168.8.100] 
```

{% hint style="warning" %}
Need to test this step with the static ip stuff - works adding it after initial setup, but needs a monitor and keyboard or access to the router to find the ip to ssh in.

Also probably want to keep the Ethernet bits in as it will likely be useful.
{% endhint %}

4. Edit 'user-data' appending the additional lines shown below. Again, use spaces, not tabs and mind the indentation. 

```text
##Reboot after cloud-init completes
power_state:
  mode: reboot
```

5. Eject the SD Card, remove it from the computer used to image it, and then insert into the powered off Raspberry Pi. Then, power up the RPi. 

6. Allow Ubuntu to boot; DO NOT try to log into Ubuntu as soon as possible. Wait until Cloud-Init runs \(although it appears to be doing nothing - in about two minutes it will show SSH info when done\). If you don't wait you may not be able to logon with the default user and passwd. At the end of the cloud-init,Ubuntu will be rebooted. What a couple of minutes for the server to boot. 

7. Logon either at the console or remotely using ssh [ubuntu@x.x.x.x](mailto:ubuntu@x.x.x.x) \(get the address from your router, etc.\) 

That's it: Headless, Wi-Fi'd Ubuntu 20.04 on Raspbery Pi

## Install ROS2 Foxy

Official instructions here: [https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) 

**But since we're installing on a current Ubuntu 20.04 server image, we just need to follow these  simple steps:** 

\(from [Install ROS 2 Foxy Fitzroy on a Raspberry Pi](https://www.youtube.com/watch?v=0w-CRiuuiKk)\) 

1. add the key:  `sudo apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc`
2. add the repository: `sudo apt-add-repository 'http://packages.ros.org/ros2/ubuntu'`
3. and install \(base = no gui stuff\):  `sudo apt install ros-foxy-ros-base`
4. Source ros2 ``source /opt/ros/foxy/setup.bash```
5. Source every time terminal runs `echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc`
6. Install auto complete `sudo apt install python3-argcomplete`
7. Try `ros2 --help` to check if it is installed - if ros2 command is not recognised check that you sourced it in step 4. 
8. Install colcon common extensions and source the auto complete bash script `sudo apt install python3-colcon-common-extensions`
9. Source `colcon_cd` to easily swap to package directories. 

   `echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc`

