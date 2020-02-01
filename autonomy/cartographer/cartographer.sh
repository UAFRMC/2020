#!/bin/bash

# make clean all


file=./cartographer
if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not excutable"
    exit 1
fi

$file