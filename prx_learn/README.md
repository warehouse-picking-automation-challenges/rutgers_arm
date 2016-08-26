# prx_learn

## Description

This package is where learning modules and their dependencies 
are placed for integration with core pracsys planning and simulation

Currently the core integration is with offline trained models via
the Caffe deep learning library. Training is done outside of pracsys 
and the trained models are stored and used for prediction in planning.


## Directory Structure

*Note*: Everything is organized according to project name, which should be
intuitively titled

__data__ / __*proj-name*__ : location of data normalization files, e.g. mean, 
min, and max values for a given dataset. Should be stored in binaryproto format

__models__ / __*proj-name*__ : location of model files, e.g. network deploy structure
(.prototxt) and trained weights (.caffemodel)

__external__ : location of external dependencies, currently the location of the 
Caffe source code


## Installation

From prx_learn root directory, we'd like a specific version of Caffe:

```
cd external/
git clone https://github.com/BVLC/caffe
cd caffe/
git reset --hard f1cc905
cp Makefile.config.example Makefile.config
```

Install Caffe's dependencies:
```
sudo apt-get install protobuf-compiler libprotobuf-dev libgoogle-glog-dev libgflags-dev libhdf5-dev libsnappy-dev liblmdb-dev libleveldb-dev libatlas-base-dev
```

CUDA is an *optional* dependency for the Caffe package. If you'd like to use
CUDA acceleration, visit [CUDA installation full documentation](http://docs.nvidia.com/cuda/cuda-getting-started-guide-for-linux/#axzz3zsjNwomC)
and see section 2.

Before compiling, in `Makefile.config` you should uncomment the line `CPU_ONLY := 1`
if you would like to run without CUDA acceleration. 
In this file you should also specify the location and type of your BLAS library

One this is done, just run make:
```
make all
make distribute
```

If you want to test your installation of CAFFE:

```
make test
make runtest
```
