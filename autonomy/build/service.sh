#!/bin/bash
#  This script is run by each service's start script, with
#     file= program to run
#     args= command line arguments for the program

if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not executable"
    exit 1
fi

# Run $file with $args
exec $file "$args" >>$file.log 2>&1
