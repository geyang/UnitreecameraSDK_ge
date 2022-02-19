#!/bin/bash
kill $(ps aux |grep autostart/camerarosnode | awk '{print $2}')
kill -9 $(ps aux |grep ros_ | awk '{print $2}')