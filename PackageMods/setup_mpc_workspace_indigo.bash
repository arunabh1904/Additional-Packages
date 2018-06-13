cd ros_ws/src/
git clone https://github.com/husky/husky.git
git clone https://github.com/husky/husky_robot.git 
git clone https://github.com/husky/husky_customization.git 
git clone https://github.com/husky/husky_desktop.git 
git clone https://github.com/husky/husky_simulator.git 
git clone https://github.com/cra-ros-pkg/robot_localization.git t
git clone https://github.com/ros-drivers/velodyne.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=indigo -y -r
catkin_make
cp src/PackageMods/launch/control_modified.launch src/husky/husky_control/launch/
cp src/PackageMods/launch/description_modified.launch src/husky/husky_description/launch/
cp src/PackageMods/launch/huskywithvelo_empty_world.launch src/husky_simulator/husky_gazebo/launch/
cp src/PackageMods/launch/spawn_huskywithvelo.launch src/husky_simulator/husky_gazebo/launch/
cp src/PackageMods/meshes/. src/husky/husky_description/meshes/accessories/
cp src/PackageMods/urdf/VLP-16.urdf.xacro src/husky/husky_description/urdf/accessories/
cp src/PackageMods/urdf/description_modified.xacro src/husky/husky_description/urdf/
cp src/PackageMods/urdf/huskywithvelo.urdf.xacro src/husky/husky_description/urdf/
cp src/PackageMods/urdf/description_modified.gazebo.xacro src/husky/husky_simulator/husky_gazebo/urdf/
cp src/PackageMods/urdf/huskywithsick.gazebo.xacro src/husky/husky_simulator/husky_gazebo/urdf/
cp src/PackageMods/urdf/huskywithvelo.gazebo.xacro src/husky/husky_simulator/husky_gazebo/urdf/
cp src/PackageMods/urdf/sick_lms100.gazebo.xacro src/husky/husky_simulator/husky_gazebo/urdf/accessories/
cp src/PackageMods/urdf/VLP-16.gazebo.xacro src/husky/husky_simulator/husky_gazebo/urdf/accessories/
