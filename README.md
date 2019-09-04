# Megarover

Package to run Vstone MegaRover from ROS & Autoware

## Dependency

* ROS (tested with kinetic and melodic)
* mpg321
```
apt update
apt install mpg321
```

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/Meijo-TasakiLab/megarover.git
cd ..
catkin_make -j1
```

When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.
