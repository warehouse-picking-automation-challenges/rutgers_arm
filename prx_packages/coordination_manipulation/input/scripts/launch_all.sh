#!/bin/bash

pushd $1

for f in *.launch;
do
	roslaunch $f
done

popd