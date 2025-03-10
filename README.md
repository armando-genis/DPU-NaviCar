# DPU-NaviCar
This repository contains packages for the autonomous vehicle project at Kütahya Dumlupınar Üniversitesi. It includes essential modules for mapping, localization, and navigation systems, enabling autonomous operation


## 📥 Installation

Before running the project, make sure you have the required dependencies installed:


sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-ros-pkgs


export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/genis/Desktop/dpu_ws/src/DPU-NaviCar/city_simulation
echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/genis/Desktop/dpu_ws/src/DPU-NaviCar/city_simulation" >> ~/.bashrc
source ~/.bashrc


## → 🛣️ Considerations for Creating HD Maps with Vector Map Builder

When creating a `Lanelet2Map` in the Vector Map Builder, follow these steps to configure the map projection:

1. Click on **Change Map Project Info**.
2. Select **Set MGRS from Lat/Lon** and input the following coordinates:
   - **Latitude:** `49`
   - **Longitude:** `8.4`
3. Click **Convert** to apply these settings.

> **Note:** When exporting the map, you may encounter an error indicating that the component `x` or `y` is negative. This error can be safely ignored, as it does not impact the map creation process. Proceed with creating the map even if these errors appear.

when finding black spaces in the rout is beacuse the lack of points you can add points with insert point with linestring. 

It's crucial to define a **centerline** for each lanelet. Here's how to create centerlines using the **TIER IV Vector Map Builder**:

1. **Select the Lanelet** ➝ Click on the lanelet for which you want to create a centerline.
2. **Access Actions Menu** ➝ In the top-right corner, click on **Action**.
3. **Create Centerline** ➝ From the dropdown, select **Create Centerline**.


## ⚙️ Scaling the Robot Model

To scale the robot correctly, modify these files:

- **`DasAutonomeAuto.urdf`** (Line 5)
    
    ```xml
    <xacro:property name="scale_percent" value="-50" />
    ```
    
- **`AckermannPlugIn.xacro`** (Line 5)
    
    ```xml
    <xacro:property name="scale_percent" value="-50"/>
    ```

## 🔄 Building Required Packages

Clone the necessary package:

```bash
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
```

Make sure you have the required dependencies installed:

```bash
sudo apt update
sudo apt-get install libeigen3-dev
sudo apt install libpcl-dev
sudo apt-get install libpcap-dev
sudo apt install can-utils
sudo apt-get install libqt5serialport5-dev
sudo apt-get install libpugixml-dev
sudo apt-get install libgeographic-dev geographiclib-tools


#ros2 packages
sudo apt install ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt-get install ros-$ROS_DISTRO-pcl-ros
sudo apt install ros-$ROS_DISTRO-vision-msgs
sudo apt install ros-$ROS_DISTRO-perception-pcl
sudo apt install ros-$ROS_DISTRO-pcl-msgs
sudo apt install ros-$ROS_DISTRO-vision-opencv
sudo apt install ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-velodyne-msgs
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
sudo apt install ros-$ROS_DISTRO-color-util

```

Then, build the required ROS2 packages:

```bash
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
```


## 🏎️ Launching the Robot Model in Gazebo

Start by launching the robot model in the Gazebo simulation environment:
```bash
ros2 launch niagara_model display.launch.py
```

## 🗺️ Visualizing the Map with `osm_visualizer`

Before starting RViz2, it's essential to visualize the map using the `osm_visualizer` node. This step is crucial because certain messages, such as `/hd_map` and `/crosswalk_polygons`, are published only once. Launch the `osm_visualizer` with the following command:

```bash
ros2 launch map_visualizer osm_visualizer.launch.py
```

**Important:** Modify the `osm_visualizer.launch.py` file to specify the correct path to your `.osm` (OpenStreetMap) file. This ensures that the visualizer loads the appropriate map data.

---

## 🛠️ Configuring Waypoints Routing

To set up the waypoints routing, update the `params.yaml` configuration file with the correct paths to your `.osm` file, as well as the desired start and end points. Once configured, launch the waypoints routing node:

```bash
ros2 launch waypoints_routing waypoints.launch.py
```

This step enables the system to calculate and follow the specified routes based on the provided waypoints.