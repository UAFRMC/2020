#!/bin/bash

../build/wait_for_X.sh

make

export file=./backend
export args=--sim

exec ../build/service.sh

