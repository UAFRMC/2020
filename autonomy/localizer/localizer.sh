#!/bin/bash

file=./localizer
if [ ! -f "$file" ]; 
then
    echo "$file does not exist"
    exit 1
elif [ ! -x "$file" ]; 
    then  
    echo "$file is not excutable"
    exit 1
fi

$file >$file.log 2>&1