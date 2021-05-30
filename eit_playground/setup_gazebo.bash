#!/bin/bash

# Save the name of the usr
usr_name=$(id -un) #$getent passwd | grep '/home' | cut -d: -f1 | tail -n 1
#echo $usr_name
if  [ $usr_name = 'nikopihl' ];
then
	echo The user is $usr_name
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Documents/8th_semester/PX4sim/Firmware
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Documents/8th_semester/PX4sim/Firmware/Tools/sitl_gazebo
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/src/EiT_group5/eit_playground/models
elif [ $usr_name = 'kenni' ];
then
	echo The user is $usr_name
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4_SITL_EIT/Firmware
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4_SITL_EIT/Firmware/Tools/sitl_gazebo
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/src/EiT_group5/eit_playground/models
elif [ $usr_name = 'marcus' ];
then
	echo The user is $usr_name
        source /home/$USER/PX4_Firmware/Tools/setup_gazebo.bash /home/$USER/PX4_Firmware /home/$USER/PX4_Firmware/build/px4_sitl_default
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4_Firmware
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/PX4_Firmware/Tools/sitl_gazebo
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/src/EiT_group5/eit_playground/models
else
	echo setup is /home	
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo
	export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PWD/src/EiT_group5/eit_playground/models
fi


echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH
