# rutgers_arm

Rutgers APC Repository

This is the set of code used by the Rutgers University ARM team for the second APC in Leipzig. 


## Installing ROS Indigo

On Ubuntu 14.04 (catkin_version):

1) Follow ROS instructions for installing Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu).

2) Install other ROS related packages

    sudo apt-get install ros-indigo-cmake-modules ros-indigo-fcl

## Install Pracsys and its dependencies

1) Get the PRACSYS repository

2) Get the PRACSYS Models repository (already included in the repo)

3) Copy the following commands to your ~/.bashrc:

    #ROS
    source /opt/ros/indigo/setup.bash
    export LIBRARY_PATH=$LIBRARY_PATH:/opt/ros/indigo/lib/
    export ROS_PARALLEL_JOBS=-j4

    #PRACSYS
    export PRACSYS_PATH=........ /pracsys/src
    export PRACSYS_MODELS_PATH=....... /pracsys_models   
    export ROS_PACKAGE_PATH=$PRACSYS_PATH:$ROS_PACKAGE_PATH
    source $PRACSYS_PATH/../devel/setup.sh

    #OSG
    export OSG_FILE_PATH=$PRACSYS_MODELS_PATH
    export OSG_LIBRARY_PATH=.:/usr/local/lib:/opt/local/lib/
    export PKG_CONFIG_PATH=/usr/X11/lib/pkgconfig:$PKG_CONFIG_PATH
    export COLLADA_BOOST_FILESYSTEM_LIBRARY=/usr/local/lib

   Make sure to edit appropriately the path to the pracsys repository in the above lines. Then execute:

    source ~/.bashrc

4) Initialize catkin workspace

    cd src
    rm -rf CMakeLists.txt
    catkin_init_workspace 

5) Install OpenSceneGraph

    sudo apt-get install libopenscenegraph-dev

6) Install Lapack

    sudo apt-get install liblapack-dev 


7) Install FCL

    sudo apt-get install ros-indigo-fcl

8) Install TRAC_IK

One of the more recent contributions is a new library for providing inverse kinematics for manipulators. The code (available at https://bitbucket.org/traclabs/trac_ik) extends KDL's IK for higher success rates.

To use this library, first install NLOpt

    sudo apt-get install libnlopt-dev

Then, you can download the trac_ik ROS package into your catkin_workspace

    git clone https://USERNAME@bitbucket.org/traclabs/trac_ik.git

Then, run these command to exclude the MoveIt specific packages

    cd trac_ik/trac_ik_kinematics_plugin
    touch CATKIN_IGNORE
    cd ../trac_ik/trac_ik_examples
    touch CATKIN_IGNORE
    cd ..

9) Configure PRACSYS

    cd prx_build
    python build_package_list.py
    gedit packages.config

10) Edit the *packages.config* to load the packages that you need (0 means the package will not be compiled, 1 means the package will be compiled).

Sample *packages.config* file

    ('rearrangement_manipulation',0)
    ('apc',1)
    ('manipulation',0)
    ('pebble_motion',0)
    ('conformant_planning',0)
    ('coordination_manipulation',0)
    ('physics_based',0)
    ('cloud_manipulation',0)
    ('two_dim_problems',0)
    ('many_arm',0)
    ('kinematic_planners',0)
    ('homotopies',0)
    ('kinodynamic',0)
    ('decentralized_coordination',0)
    ('crowd_simulation',0)

11) Build external dependencies:

    python catkin_compile_packages.py
    cd ../prx_external 
    cmake .
    make

12) Build PRACSYS

    cd $PRACSYS_PATH/.. && catkin_make


