cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(asv_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# ---------------------------------------------------------------------
# Environment hooks.
# Tanpa ini Gazebo tidak tahu di mana folder mesh berada dan model
# muncul tanpa bentuk (atau gagal diam-diam). Pola ini disalin dari
# wamv_description milik VRX.
# ---------------------------------------------------------------------
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# ---------------------------------------------------------------------
# Validasi xacro saat build.
# Kalau ada error sintaks atau $(arg ...) yang belum dideklarasikan,
# colcon build langsung gagal di sini - jauh lebih cepat ketahuan
# daripada menunggu spawn di Gazebo.
# Keluarannya terpasang sebagai share/asv_description/urdf/asv.urdf
# ---------------------------------------------------------------------
xacro_add_files(
  urdf/asv.urdf.xacro
  INSTALL DESTINATION urdf
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
