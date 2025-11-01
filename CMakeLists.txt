cmake_minimum_required(VERSION 3.8) 
project(lab1_urdf) 
 
find_package(ament_cmake REQUIRED) 
 
# Install non-source assets (launch/urdf/rviz) into the share directory 
install(DIRECTORY launch urdf rviz 
  DESTINATION share/${PROJECT_NAME} 
) 
 
ament_package() 