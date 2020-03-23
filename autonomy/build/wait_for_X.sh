#!/bin/sh

# FIXME: it's :1 on my Ubuntu laptop; :0 on the pi
[ -z ${DISPLAY+x} ] && export DISPLAY=:0   

echo "Trying to connect to X on $DISPLAY"
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


