cmake_minimum_required(VERSION 3.5)
project(decomp_basis)

find_package(ament_cmake REQUIRED)

# install all headers under include/decomp_basis
install(
  DIRECTORY include/
  DESTINATION include
)

# tell ament to generate decomp_basisConfig.cmake
ament_export_include_directories(include)

ament_package()
