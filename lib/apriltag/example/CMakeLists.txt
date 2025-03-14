# Examples
# apriltag_demo
add_executable(apriltag_demo apriltag_demo.c)
target_link_libraries(apriltag_demo ${PROJECT_NAME})

# opencv_demo
set(_OpenCV_REQUIRED_COMPONENTS core imgproc videoio highgui)
find_package(
  OpenCV
  COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS}
  QUIET CONFIG)
if(OpenCV_FOUND)
  # NB: contrib required for TickMeter in OpenCV 2.4. This is only required for
  # 16.04 backwards compatibility and can be removed in the future. If we add it
  # to the find_package initially, the demo won't build for newer OpenCV
  # versions
  if(OpenCV_VERSION VERSION_LESS "3.0.0")
    list(APPEND _OpenCV_REQUIRED_COMPONENTS contrib)
    find_package(OpenCV COMPONENTS ${_OpenCV_REQUIRED_COMPONENTS} CONFIG)
  endif()

  add_executable(opencv_demo opencv_demo.cc)
  target_link_libraries(opencv_demo ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  set_target_properties(opencv_demo PROPERTIES CXX_STANDARD 11)
  install(TARGETS opencv_demo RUNTIME DESTINATION bin)
else()
  message(STATUS "OpenCV not found: Not building demo")
endif(OpenCV_FOUND)

