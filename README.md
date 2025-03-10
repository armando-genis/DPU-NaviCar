# DPU-NaviCar
This repository contains packages for the autonomous vehicle project at Kütahya Dumlupınar Üniversitesi. It includes essential modules for mapping, localization, and navigation systems, enabling autonomous operation


sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-ros-pkgs


export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/genis/Desktop/dpu_ws/src/DPU-NaviCar/city_simulation
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/genis/Desktop/dpu_ws/src/DPU-NaviCar/city_simulation" >> ~/.bashrc
source ~/.bashrc


scale robot
    DasAutonomeAuto.urdf line 5
  <xacro:property name="scale_percent" value="-50" />
    AckermannPlugIn.xacro line 5
    <xacro:property name="scale_percent" value="-50"/>


    git clone https://github.com/KIT-MRT/mrt_cmake_modules.git

colcon build --packages-select polygon_msgs
colcon build --packages-select mrt_cmake_modules
source install/setup.bash
colcon build --packages-select map_visualizer
colcon build --packages-select lanelet2_core
colcon build --packages-select lanelet2_maps
colcon build --packages-select lanelet2_io
colcon build --packages-select lanelet2_projection
colcon build --packages-select lanelet2_traffic_rules
colcon build --packages-select lanelet2_routing
colcon build --packages-select traffic_information_msgs
colcon build --packages-select lanelet2_validation
colcon build --packages-select target_waypoint_index
source install/setup.bash
colcon build


robot and gazebo
ros2 launch niagara_model display.launch.py

run this before rviz2 becuase the messages only send only once /hd_map and /crosswalk_polygons:
also in the launch file change the path to the oms file
ros2 launch map_visualizer osm_visualizer.launch.py

change the parametns of the params.yaml with the correct path osm file and start and end point:
ros2 launch waypoints_routing waypoints.launch.py


