cmake_minimum_required(VERSION 2.8)
project(RobotGraspRender)

# turn on c++11
add_definitions(-std=c++11)

set(BUILD_SHARED_LIBS true)

find_package(PkgConfig)

# Pangolin
find_package(Pangolin 0.5 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Assimp
pkg_check_modules(assimp REQUIRED assimp)
include_directories(${assimp_INCLUDE_DIRS})

# KDL
pkg_check_modules(orocos_kdl REQUIRED orocos_kdl)
include_directories(${orocos_kdl_INCLUDE_DIRS})

# URDF
find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS})

# Boost filesystem
find_package(Boost REQUIRED filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

# LCM
pkg_check_modules(LCM REQUIRED lcm)
include_directories(${lcm_INCLUDE_DIRS})
# LCM types
include_directories(lcmtypes_cpp)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# project headers
include_directories(src)

# sources for renderer
set(SRC_LIST ./src/MeshLoader.cpp
             ./src/Mesh.cpp)

set(HDR_LIST ./src/MeshLoader.hpp
             ./src/Mesh.hpp)

# loading meshes
add_library(meshloader ${SRC_LIST})
target_link_libraries(meshloader ${assimp_LIBRARIES})

# parser for generating KDL tree from URDF
add_library(kdl_parser src/kdl_parser.cpp)

# represenation of articulated robot model
add_library(robot_model src/RobotModel.cpp)
target_link_libraries(robot_model orocos-kdl)
target_link_libraries(robot_model ${urdfdom_LIBRARIES})
target_link_libraries(robot_model kdl_parser)
target_link_libraries(robot_model ${Boost_LIBRARIES})


# define executable
add_executable(${PROJECT_NAME} ./src/RobotGraspRender.cpp)
target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} meshloader)
target_link_libraries(${PROJECT_NAME} robot_model)
target_link_libraries(${PROJECT_NAME} ${LCM_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# install
install(TARGETS meshloader robot_model kdl_parser LIBRARY DESTINATION lib)  # shared library
install(DIRECTORY src/ DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.hpp")

SET(PKG_CONFIG_REQUIRES "pangolin assimp urdfdom orocos_kdl")
SET(PKG_CONFIG_LIBDIR       "\${prefix}/lib" )
SET(PKG_CONFIG_INCLUDEDIR   "\${prefix}/include" )
SET(PKG_CONFIG_LIBS         "-L\${libdir} -lrobot_model -lkdl_parser -lmeshloader" )
SET(PKG_CONFIG_CFLAGS       "-I\${includedir}" )

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/pkg-config.pc.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
)

INSTALL(FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
        DESTINATION lib/pkgconfig)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()
