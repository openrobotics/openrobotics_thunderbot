Compilation:
First, cmake .
Then, make
Then, rosmake


Usage:
roslaunch tbots_navigation thunderbot_bringup.launch


Description:
The above brings up all of the static transforms necessary for the navigation stack except for the LIDAR, those haven't been added yet.

It should also run the base_controller_node, which will only work if the computer is connected to the arduino AND you have sudo permissions on tty/ACM0. (If you get that issue, type this: sudo chmod a+rwx /dev/


