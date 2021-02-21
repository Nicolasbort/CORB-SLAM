cmake_minimum_required(VERSION 3.0.2)
project(corbslam_server)


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


list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/cmake/Modules")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf
        message_generation
)



find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
        find_package(OpenCV 3 QUIET)
        if(NOT OpenCV_FOUND)
                message(FATAL_ERROR "OpenCV > 3 not found.")
        endif()
endif()
message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")

find_package(Pangolin REQUIRED)
#find_package(corbslam_client REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost 1.71.0 COMPONENTS serialization system filesystem REQUIRED)
if (NOT Boost_FOUND)
        find_package(Boost COMPONENTS serialization system filesystem REQUIRED)
        if (NOT Boost_FOUND)
                message(FATAL_ERROR "Boost not found.")
        endif()
endif()
message(STATUS "Boost ${Boost_VERSION} FOUND")

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
        #  INCLUDE_DIRS include
        #  LIBRARIES orbslam_server
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
        CATKIN_DEPENDS message_runtime corbslam_client
)

###########
## Build ##
###########

message(STATUS "SOURCE DIR ${CMAKE_SOURCE_DIR}")
message(STATUS "PROJECT SOURCE DIR ${PROJECT_SOURCE_DIR}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client
        ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/include
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        /usr/include/pcl-1.10

)

link_directories(/usr/local/lib)

set(${PROJECT_NAME}_SOURCES
        ${PROJECT_SOURCE_DIR}/src/MapFusion.cpp
        ${PROJECT_SOURCE_DIR}/src/main.cpp
        ${PROJECT_SOURCE_DIR}/src/ServerMap.cpp
        ${PROJECT_SOURCE_DIR}/src/ServerMapView.cpp
        ${PROJECT_SOURCE_DIR}/src/PubToClient.cpp
        ${PROJECT_SOURCE_DIR}/src/GlobalOptimize.cpp
        )


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

message(STATUS "CORBSLAM_CLIENT LIBS ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/lib/")

add_executable(${PROJECT_NAME}
        ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NAME} corbslam_client)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/Thirdparty/DBoW2/lib/libDBoW2.so
        ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/Thirdparty/g2o/lib/libg2o.so
        ${CMAKE_SOURCE_DIR}/CORB-SLAM/corbslam_client/lib/libcorbslam_client.so
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        #${ODB_INCLUDE_DIRS}
        #${ODB_COMPILE_OUTPUT_DIR}
)
target_compile_definitions(${PROJECT_NAME}
        PRIVATE
        #DATABASE_PGSQL
)
