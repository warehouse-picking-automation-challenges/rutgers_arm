#!/bin/bash

N=1
COUNTER=0

while read line
do
    echo $line
    COUNTER=0
    while [  $COUNTER -lt $N ]; do
        echo 1
        job.sh $line
        let COUNTER=COUNTER+1 
    done 
done < $1