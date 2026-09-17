cmake_minimum_required(VERSION 3.5)
project(srb)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Install Python package
ament_python_install_package(srb)

## Install CLI script
install(PROGRAMS
    srb/__main__.py
    DESTINATION lib/${PROJECT_NAME}
)

## Alias the CLI as "main"
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink \"__main__.py\" \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/main\")")

## Setup the project
ament_package()
