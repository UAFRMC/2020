#!/bin/sh

export DISPLAY=:1   # FIXME: what is it on the pi?

echo "Trying to find X"
while [ true ]
do
    xdpyinfo > /dev/null 2>&1
    if [ $? -eq 0 ]
    then
        echo "Found X"
        exit 0 # X exists
    fi
    # Otherwise wait
    sleep 1
done


