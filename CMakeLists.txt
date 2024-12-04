# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(gps_ublox
        VERSION 0.1
        DESCRIPTION "Driver for Ublox GNSS modules")
find_package(Rock)
set(CMAKE_CXX_STANDARD 17)
rock_init()
rock_standard_layout()
