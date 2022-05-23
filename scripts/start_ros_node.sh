#!/bin/bash
export ROS_MASTER_URI=http://192.168.123.161:11311
#../bins/ros_RawFrame_server
../bins/ros_RectFrame_server 0 &
#../bins/ros_RectFrame_server 1 &
#../bins/ros_DepthFrame_server
