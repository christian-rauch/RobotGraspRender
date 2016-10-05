cmake_minimum_required(VERSION 2.8)
project(RobotGraspRender)

# turn on c++11
add_definitions(-std=c++11)

find_package(PkgConfig)

# Pangolin
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Assimp
pkg_check_modules(assimp REQUIRED assimp)
include_directories(${assimp_INCLUDE_DIRS})

# project headers
include_directories(src)

# sources for renderer
set(SRC_LIST ./src/MeshLoader.cpp)

set(HDR_LIST ./src/MeshLoader.hpp)

add_library(meshloader ${SRC_LIST})
target_link_libraries(meshloader ${assimp_LIBRARIES})

# define executable
add_executable(${PROJECT_NAME} ./src/RobotGraspRender.cpp)
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} meshloader)
