cmake_minimum_required(VERSION 3.10)
project(xgc_camera_driver VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  image_transport
  roscpp
  sensor_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
find_package(xgc2_camera CONFIG REQUIRED)

catkin_package(
  CATKIN_DEPENDS camera_info_manager cv_bridge diagnostic_msgs diagnostic_updater image_transport roscpp sensor_msgs
  DEPENDS OpenCV
)

add_executable(xgc_camera_driver_node src/camera_driver_node.cpp)
target_compile_features(xgc_camera_driver_node PRIVATE cxx_std_14)
target_compile_options(xgc_camera_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(xgc_camera_driver_node
  PRIVATE
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    PkgConfig::FFMPEG
    xgc2::camera
)
target_include_directories(xgc_camera_driver_node PRIVATE ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

install(TARGETS xgc_camera_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../process-definitions/xgc2-camera-ros1.json
  DESTINATION /usr/share/xgc2/process-definitions
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_h264_input_buffer test/test_h264_input_buffer.cpp)
  if(TARGET test_h264_input_buffer)
    target_compile_features(test_h264_input_buffer PRIVATE cxx_std_14)
    target_include_directories(test_h264_input_buffer PRIVATE src)
  endif()
  add_rostest(test/synthetic_driver.test)
endif()
