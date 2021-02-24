cmake_minimum_required(VERSION 3.0.2)
project(corbslam_client)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native ")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC14)
    message(STATUS "Using flag -std=c++14.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif ()


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        image_transport
        tf
        message_generation
)

# OpenCV
find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
        find_package(OpenCV 3 QUIET)
        if(NOT OpenCV_FOUND)
                message(FATAL_ERROR "OpenCV > 3 not found.")
        endif()
endif()
message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")

find_package(Pangolin REQUIRED)

# Boost
find_package(Boost 1.71.0 COMPONENTS serialization system filesystem REQUIRED)
if (NOT Boost_FOUND)
        find_package(Boost COMPONENTS serialization system filesystem REQUIRED)
        if (NOT Boost_FOUND)
                message(FATAL_ERROR "Boost not found.")
        endif()
endif()
message(STATUS "Boost ${Boost_VERSION} FOUND")

# Eigen
SET( EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3" )
IF( NOT EIGEN3_INCLUDE_DIR )
        SET( EIGEN3_INCLUDE_DIR "/usr/include/eigen3" )
        IF( NOT EIGEN3_INCLUDE_DIR )
                MESSAGE( FATAL_ERROR "Eigen3 Include Dir not found")
        endif()
ENDIF()
message(STATUS "Eigen FOUND")


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        corbslam_message.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        corbslam_insert.srv
        corbslam_update.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/Examples/RGB-D
        /usr/include/pcl-1.10
)

## Declare a C++ library

link_directories(/usr/local/lib)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        include/Cache.h
        src/Cache.cc
        include/LightKeyFrame.h
        src/LightKeyFrame.cc
        include/LightMapPoint.h
        src/LightMapPoint.cc
        include/SerializeObject.h
        include/DataDriver.h
        src/DataDriver.cc
        include/TransPose.h src/TransPose.cpp)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${Boost_LIBRARIES}
)



# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(corbslam_client_rgbd_tum
#         Examples/RGB-D/rgbd_tum.cc
#         Examples/RGB-D/ros_viewer.cpp)

# target_link_libraries(corbslam_client_rgbd_tum
#         ${catkin_LIBRARIES}
#         ${OpenCV_LIBS}
#         ${PROJECT_NAME}
#         )

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(corbslam_client_stereo_kitti Examples/Stereo/stereo_kitti.cc)

# target_link_libraries(corbslam_client_stereo_kitti
#         ${catkin_LIBRARIES}
#         ${OpenCV_LIBS}f
#         ${PROJECT_NAME})

#add_executable(orbslam_client_node_stereo_euroc
#        Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(orbslam_client_node_stereo_euroc ${PROJECT_NAME})
#


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(corbslam_client_mono_tum Examples/Monocular/mono_tum.cc)

# target_link_libraries(corbslam_client_mono_tum
#         ${PROJECT_NAME}
#         ${OpenCV_LIBS}
#         ${catkin_LIBRARIES}
# )



add_executable(corbslam_client_mono_kitti Examples/Monocular/mono_kitti.cc)

target_link_libraries(corbslam_client_mono_kitti
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
)


# add_executable(corbslam_client_mono_ros Examples/Monocular/mono_ros.cc)

# target_link_libraries(corbslam_client_mono_ros
#         ${PROJECT_NAME}
#         ${OpenCV_LIBS}
#         ${catkin_LIBRARIES}
# )