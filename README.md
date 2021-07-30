# Dynamic Object Mapping for ROS

Extract dynamic object information from autonomous vehicle sensor data

## Prerequisties

- Ubuntu 18.04, ROS Melodic
- GPU: RTX 2060 Super / GTX 1650 Ti

## Building
```
$ cd ~/src
$ git clone --recursive http://github.com/shinsoo0203/object_mapping.git
$ git submodule update --init --recursive
```

## Submodules
```
$ git submodule add [forked submodule git address]
$ git checkout melodic
```
- Darknet_ros

```
$ git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
$ catkin_make -DCMAKE_BUILD_TYPE=Release
$ sudo gedit CMAKELists.txt
-gencode arch=compute_75,code=sm_75
```
- gb_visual_detection_3d / gb_visual_detection_3d_msgs
