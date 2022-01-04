# Run on Pi startup

### To Run pi cam on startup:&#x20;

  The command has to be executed after the desktop has loaded or it will not be visible on some displays (i.e dji controllers).&#x20;

So use this guide:&#x20;

[https://bigl.es/tooling-tuesday-auto-start-a-gui-application-in-raspbian/](https://bigl.es/tooling-tuesday-auto-start-a-gui-application-in-raspbian/)&#x20;

  The basic command to preview the pi cam is;&#x20;

`raspistill -t 0`&#x20;

  There are a range of optional parameters that can be used to change the aspect ratio, size etc. &#x20;

[https://www.raspberrypi.org/documentation/raspbian/applications/camera.md](https://www.raspberrypi.org/documentation/raspbian/applications/camera.md)&#x20;

If you want to capture the video then the raspistill command probably isn't the correct one - check the link above for documentation.&#x20;



### Push ip address to phone on startup&#x20;

{% embed url="https://www.raspberrypi.org/forums/viewtopic.php?t=79151" %}

### Using crontab is an easy way to schedule scripts&#x20;

{% embed url="https://www.raspberrypi.org/blog/how-to-run-a-script-at-start-up-on-a-raspberry-pi-using-crontab" %}

&#x20;

{% embed url="https://answers.ros.org/question/333968/how-to-start-ros2-node-automatically-after-starting-the-system" %}
