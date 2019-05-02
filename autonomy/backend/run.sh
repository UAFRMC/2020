#!/bin/sh
export DISPLAY=:0
export BEACON=10.10.10.100
./backend "$@"
