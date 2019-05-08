#!/bin/sh
killall realsense
cd `dirname $0`
./realsense --nogui >> log 2>&1 &
disown

