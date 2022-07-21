#!/bin/bash
kill $(ps aux |grep autostart | awk '{print $2}')
kill -9 $(ps aux |grep ros_ | awk '{print $2}')

sudo ifconfig eth0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0

/scratch/gmargo/UnitreecameraSDK/bins/example_getRawFrameLCM 0 &
/scratch/gmargo/UnitreecameraSDK/bins/example_getRawFrameLCM 1 &
