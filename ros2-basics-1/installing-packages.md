# Installing Packages

## Official \(?\) released packages

If there is a release we can use apt-get install, otherwise need to clone the git repo and build from source.

[https://industrial-training-master.readthedocs.io/en/melodic/\_source/session1/Installing-Existing-Packages.html](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Installing-Existing-Packages.html)

```text
$ sudo apt-get install ros-<ros_version>-PACKAGE
```

## Git Repos

Instruction here:

{% embed url="https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/\#clone-a-sample-repo" %}

1. Go to [github](http://github.com/search).
2. Search for your package
3. Click on this repository, and look to the right for the _Clone or Download_, then copy to clipboard.
4. clone to src folder:

```text
$ cd ros2_ws/src
$ git clone <path to git repo>
```

Often there are different branches for different ROS distros, with the master not being the one we are wanting to use \(foxy\) so you can just clone a single branch.

```text
git clone --single-branch --branch <branchname> <remote-repo>
```

> You can just use the -b or --branch argument but this downloads the whole repo and checkouts the branch you specify, which seems unnecessary

5.  Best practice is to check for dependencies every time you clone

```text
# at workspace root (ros2_ws)
rosdep install -i --from-path src --rosdistro foxy -y
```

6. Then `colcon build` remember you can use `--packages-select`  or --packages-up-to to inc dependencies.

7. Remember you need to `source ~/ros2_ws/install/local_setup.bash` \(which should be in /.bashrc\) in other terminal windows to use newly built packages.

