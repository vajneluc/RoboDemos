#!/bin/bash
SRCDIR=~/devel/RoboDemos/scripts
for f in `ls $SRCDIR` 
do 
    echo $f
    ln -s $SRCDIR/$f ~/bin
done
