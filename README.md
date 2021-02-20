# A ROS/Gazebo simulation for px4 UAV landing on mobile car based on vision


**1.**roslaunch px4 px4_car.launch  
Start the gazebo simulation and start the gimbal control.  
**px4_car.launch should be in the Launch folder of Firmware.**

Firmware is installed as follows：  
git clone https://github.com/PX4/PX4-Autopilot.git  
mv PX4-Autopilot Firmware  
cd Firmware  
git checkout -b xtdrone/dev v1.11.0-beta1  
git submodule update --init --recursive  
make px4_sitl_default gazebo  


**2.**roslaunch mobot_urdf car_tracking.launch  
Start ar_track_alvar and control node. **Pay attention to the topic name of the camera**  



video: https://www.youtube.com/watch?v=vf8mISHC7H0  


**Before using, please check the name and path of all packages.  

**Check the name and path of the node in all launch files.**  

**Check the quality of all packages CmakeList.txt Whether to delete or add nodes in**.  



If there is any problem in the project, please refer to Issues. I will add or modify the document and add the description immediately.  
