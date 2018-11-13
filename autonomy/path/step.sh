#!/bin/sh
# Drive stepper motor connected to Raspberry Pi 3

g="gpio -g "
$g mode 2 out
$g mode 3 out
$g mode 4 out
$g mode 17 out

step() {
	$g write 2 $1
	shift
	$g write 4 $1
	shift
	$g write 3 $1
	shift
	$g write 17 $1
	shift
	sleep 0.002
}

cw() {
step 1 0 1 0
step 0 1 1 0
step 0 1 0 1
step 1 0 0 1
}

ccw() {
step 1 0 0 1
step 0 1 0 1
step 0 1 1 0
step 1 0 1 0
}

i="$1"
while [ "$i" -gt 0 ]
do
	echo "Step $i"
	cw
	i=`expr "$i" - 1`
done
while [ "$i" -lt 0 ]
do
	echo "Step $i"
	ccw
	i=`expr "$i" + 1`
done
step 0 0 0 0

