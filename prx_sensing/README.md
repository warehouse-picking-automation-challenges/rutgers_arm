# prx_sensing

## Description

Location of all external packages, internal libraries, launches, and executables
relating to any hardware interfaces for sensing (vision, tactile, force/torque, etc).

*Note:* Installation of this package both assumes and **requires** prior installation
of a modern version of CUDA acceleration.

## Directory Structure

__external__ : location of external dependencies, currently the location of the 
Simtrack and its minimal dependencies

## Installation

First, you need to choose to build the prx_sensing package:
```
cd prx_build
python catkin_compile_packages.py Debug Sense
cd ..
```

Then from the root directory, we start by installing Simtrack:

```
cd prx_sensing/external/
git clone https://github.com/karlpauwels/simtrack.git simtrack
git clone https://github.com/NVlabs/cub.git --branch 1.4.1 cub

```

And that's all for right now! Will update as we build this out.