# megarover_change_speed_angular
## How to Use
1. Connect MegaRover and PC with a USB cable.
2. Use the following commands.  
``` sudo chmod 666 /dev/ttyUSB0 ```  
3. Launch ```roscore```.
4. Run ```rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200``` in another terminal.
5. Run ```rosrun megarover_change_speed_angular megarover_change_speed_angular``` in another terminal.
    - Option
        - -s : Linear Speed [m/s] (default : 0.0)
        - -a : Angular velocity [rad/s] (default : 0.0)
        - -h : Print Help
