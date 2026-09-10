cmake_minimum_required(VERSION 3.5)
project(champ)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()
