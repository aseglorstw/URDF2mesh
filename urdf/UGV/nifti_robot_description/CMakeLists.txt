cmake_minimum_required(VERSION 2.8.3)
project(nifti_robot_description)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

find_package(xacro REQUIRED)
if("${xacro_VERSION}" VERSION_LESS "1.13.4")
  message(FATAL_ERROR "Please update your system pacakges. Xacro 1.13.4 and higher is required.")
endif()

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/RobotParts.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(
  PROGRAMS
    nodes/dynamic_robot_model_server.py
    scripts/print_robot_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  meshes/antenna.dae
  meshes/battery.dae
  meshes/body.dae
  meshes/bogie.dae
  meshes/flipper.dae
  meshes/laser.dae
  meshes/omnicam.dae
  meshes/top_box.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(FILES
  meshes/collide/antenna.dae
  meshes/collide/battery.dae
  meshes/collide/body.dae
  meshes/collide/bogie.dae
  meshes/collide/flipper.dae
  meshes/collide/laser.dae
  meshes/collide/omnicam.dae
  meshes/collide/top_box.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes/collide
)
install(FILES
  textures/antenna_color.png
  textures/body_color.png
  textures/bogie_color.png
  textures/flipper_color.png
  textures/laser_color.png
  textures/omnicam_color.png
  textures/top_box_color.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/textures
)
install(FILES
  launch/display.launch
  launch/load_robot_model.launch
  launch/state_publisher.launch;
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES
  rviz/urdf.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)