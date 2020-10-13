#!/bin/bash

file=./pathplanner
if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not excutable"
    exit 1
fi

# Set parameters here!
$file --lag 3000 >$file.log 2>&1