#!/bin/bash

# make clean all
make

file=./vision
if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not excutable"
    exit 1
fi

$file --gui --fps 15 >$file.log 2>&1