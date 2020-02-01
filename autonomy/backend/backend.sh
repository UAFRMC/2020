#!/bin/bash

# make clean all
make

file=./backend

if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not excutable"
    exit 1
fi

# $file
$file --sim >$file.log 2>&1