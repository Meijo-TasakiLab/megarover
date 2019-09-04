# megarover_autoware

## How to Use
1. Connect MegaRover and PC with a USB cable.
2. Use the following commands.  
``` sudo chmod 666 /dev/ttyUSB0 ```  
3. Use one of the following commands to launch "megarover_autoware".  
    - ``` roslaunch megarover_autoware megarover_autoware_AutoPilot.launch ```  
    - ``` roslaunch megarover_autoware megarover_autoware_Measurement.launch ```  
4. When "AutoPilot" is turned on in Autoware, MegaRover can be controlled from Autoware.  
![Autoware_Interface](https://github.com/Meijo-TasakiLab/megarover/blob/master/megarover_autoware/images/Autoware_Interface.png)

