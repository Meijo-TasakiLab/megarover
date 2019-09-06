# Megarover

Package to run Vstone MegaRover from ROS & Autoware

## Dependency

- ROS (tested with kinetic and melodic)
	* rosserial  
	* autoware-msgs  
	  
        (for kinetic)  
        ```
        apt install ros-kinetic-rosserial ros-kinetic-autoware-msgs
        ```  
        (for melodic)  
        ```
        apt install ros-melodic-rosserial ros-melodic-autoware-msgs
        ```  
- mpg321  
    ```
    apt install mpg321
    ```  
- [megarover_samples](https://github.com/vstoneofficial/megarover_samples.git)
    ```
    cd ~/catkin_ws/src  
    git clone https://github.com/vstoneofficial/megarover_samples.git  
    cd ..  
    catkin_make  
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
