cmake_minimum_required(VERSION 3.10)
project(cm_liuw_odometry)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3 -pthread -fexceptions)

option(BUILD_PGO "Build the optional PALoc pose-graph optimization node" OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()


# include for system output to show the sys
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()
# include for system output

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(Boost REQUIRED COMPONENTS system thread iostreams chrono)

if(BUILD_PGO)
  find_package(Open3D REQUIRED)
  find_package(GTSAM REQUIRED QUIET)
endif()

###########################################################################
# Find Boost
find_package(Boost REQUIRED COMPONENTS filesystem regex timer date_time program_options)
if (NOT TARGET Boost::boost)
    add_library(Boost::boost INTERFACE IMPORTED)
    set_target_properties(Boost::boost PROPERTIES
            INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif ()
######################################################

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
  rosconsole  # 确保包含这个库
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package (GeographicLib REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  SYSTEM
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Y_INCLUDE_DIRS}
  include)

if(BUILD_PGO)
  include_directories(${Open3D_INCLUDE_DIRS})
endif()

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS common_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs
                 std_msgs tf pcl_ros livox_ros_driver message_runtime rosconsole
  DEPENDS EIGEN3 PCL
)

add_executable(cm_liuw_slam
              #lidar
              src/laserMapping.cpp
              include/ikd-Tree/ikd_Tree.cpp
              src/preprocess.cpp
              #uwb
              src/uwb_localization/uwb_eskf.cpp
              src/uwb_localization/uwb_init_loc.cpp
              src/uwb_localization/utils.cpp
              )

add_dependencies(cm_liuw_slam
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cm_liuw_slam
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS} #OpenCV为LIBS
    ${CERES_LIBRARIES}
    Threads::Threads
    )

target_include_directories(cm_liuw_slam PRIVATE
    ${PYTHON_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS})



if(BUILD_PGO)
  add_executable(pgo_localization_ori
      src/paloc/paloc.cpp
      src/paloc/cloud_process.cpp
      src/paloc/data_saver.cpp
      include/factors/gravity_factor.h
      include/factors/gravity_kalman_filter.h
      include/base_type.cpp
  )

  add_dependencies(pgo_localization_ori
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(pgo_localization_ori
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${PYTHON_LIBRARIES}
      ${Open3D_LIBRARIES}
      ${Boost_LIBRARIES}
      ${OpenCV_LIBS}
      ${GeographicLib_LIBRARIES}
      ${CERES_LIBRARIES}
      ${OpenMP_CXX_FLAGS}
      gtsam
  )
endif()

install(TARGETS cm_liuw_slam
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(BUILD_PGO)
  install(TARGETS pgo_localization_ori
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY config launch msg rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
