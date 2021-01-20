cmake_minimum_required(VERSION 3.0.2)
project(seek)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_MODULE_PATH "${seek_SOURCE_DIR}/cmake")
include(build_type)

set (WITH_ADDRESS_SANITIZER false CACHE BOOL "Enable address sanitizer")
set (WITH_DEBUG_VERBOSITY false CACHE BOOL "Enable verbose stdout messages")
set (default_build_type "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

if (NOT MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
endif ()

if (WITH_DEBUG_VERBOSITY)
    add_definitions(-DSEEK_DEBUG)
endif ()

message (STATUS "Build type: ${CMAKE_BUILD_TYPE}")

include (MacroLogFeature)

find_package (LibUSB)
macro_log_feature (LIBUSB_FOUND "libusb" "Required to communicate via USB" "https://libusb.info/" TRUE)

find_package (OpenCV REQUIRED)
macro_log_feature (OPENCV_FOUND "OpenCV" "Required to handle image processing" "https://opencv.org/" TRUE)

if (MSVC)
	include_directories (${PROJECT_SOURCE_DIR}/win)
endif ()

# add_subdirectory (src)
# add_subdirectory (examples)

macro_display_feature_log()

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)

# add the resized image message
#add_message_files(DIRECTORY msg
#   FILES ResizedImage.msg
#)
#generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS}
${LIBUSB_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})

# include_directories (
#     ${LIBUSB_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )

set (HEADERS
src/SeekCam.h
src/SeekDevice.h
src/seek.h
src/SeekLogging.h
src/SeekThermal.h
src/SeekThermalPro.h
)

set (SOURCES
src/seek_ros.cpp
src/SeekCam.cpp
src/SeekDevice.cpp
src/SeekThermal.cpp
src/SeekThermalPro.cpp
)

set (SRC ${SOURCES} ${HEADERS})

# add the publisher example
# add_executable(seek_ros src/seek_ros.cpp)
add_executable(seek_ros ${SRC})
add_dependencies(seek_ros ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(seek_ros ${catkin_LIBRARIES}
${LIBUSB_LIBRARIES}
${OpenCV_LIBS})
# target_link_libraries(seek_ros seek_static ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
# add_executable(my_subscriber src/my_subscriber.cpp)
# add_dependencies(my_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the plugin example
# add_library(resized_publisher src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
# add_dependencies(resized_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(resized_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Mark executables and/or libraries for installation
# install(TARGETS my_publisher my_subscriber resized_publisher
install(TARGETS seek_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS image_norm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(FILES resized_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
