#!/bin/bash

# make clean all
make clean all

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

$file  --fps 6 >$file.log 2>&1
