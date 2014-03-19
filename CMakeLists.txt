cmake_minimum_required(VERSION 2.8)

project(OgreCamTest)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIR})

# Obviously windows-only, ugh.
list(APPEND CMAKE_MODULE_PATH $ENV{OGRE_HOME}/CMake)
find_package(OGRE REQUIRED)
include_directories(${OGRE_INCLUDE_DIRS})

# boost for OGRE
find_package(Boost COMPONENTS thread system date_time chrono)
include_directories(${Boost_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-DOGRE_PLUGIN_DIR_REL="${OGRE_PLUGIN_DIR_REL}")
add_definitions(-DOGRE_PLUGIN_DIR_DBG="${OGRE_PLUGIN_DIR_DBG}")

add_executable(ogre-cam-test main.cxx)
target_link_libraries(ogre-cam-test
	${SDL2_LIBRARY}
	${OGRE_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
)