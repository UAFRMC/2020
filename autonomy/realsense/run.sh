#!/bin/sh
killall realsense
cd `dirname $0`
./realsense --stepper >> ../log.txt 2>&1 &
echo "Started new realsense"
tail -10 ../log.txt

