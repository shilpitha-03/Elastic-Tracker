cmake_minimum_required(VERSION 3.5)
project(decomp_geometry)

# find the ROS 2 build system
find_package(ament_cmake REQUIRED)

# this package depends on decomp_basis
find_package(decomp_basis REQUIRED)

# install all headers under include/decomp_geometry/...
install(
  DIRECTORY include/
  DESTINATION include
)

# if downstream packages need to find your headers:
ament_export_include_directories(include)

# so that find_package(decomp_geometry) will pull in decomp_basis transitively:
ament_export_dependencies(decomp_basis)

ament_package()
