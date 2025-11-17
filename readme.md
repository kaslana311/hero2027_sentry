cd src/third_party/Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install

cd ../../livox_ros_driver2/
./build.sh humble

cd ../small_gicp
mkdir build && cd build
cmake .. && make -j
sudo make install

colcon build --packages-select sophus --symlink-install
colcon build --packages-select vikit_common --symlink-install
colcon build --packages-select vikit_ros --symlink-install

colcon build --packages-select fast_livo --symlink-install
colcon build --packages-select point_lio --symlink-install
colcon build --packages-select interfaces --symlink-install
colcon build --packages-select serial_interfaces --symlink-install

. install/setup.bash
colcon build --packages-select tf_manager --symlink-install
colcon build --packages-select fast_lio --symlink-install
colcon build --packages-select pcd2pgm --symlink-install
colcon build --packages-select fake_vel_transform --symlink-install

. install/setup.bash
colcon build --packages-select hero_serial --symlink-install

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup 
colcon build --packages-select map_server --symlink-install
colcon build --packages-select pb_omni_pid_pursuit_controller --symlink-install
colcon build --packages-select pb_nav2_plugins --symlink-install
colcon build --packages-select terrain_analysis --symlink-install
colcon build --packages-select terrain_analysis_ext --symlink-install
colcon build --packages-select pcl_tool --symlink-install

colcon build --packages-select behaviortree_cpp --symlink-install
. install/setup.bash 
colcon build --packages-select bt --symlink-install