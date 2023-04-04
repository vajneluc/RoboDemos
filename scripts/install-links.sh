#!/bin/bash
SRCDIR=/home/ros/devel/RoboDemos/scripts
for f in `ls $SRCDIR` 
do 
    echo $f
    ln -s $SRCDIR/$f /home/ros/bin
done
