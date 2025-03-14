find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d highgui)
find_package(yaml-cpp REQUIRED)

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/apriltag ${PROJECT_BINARY_DIR}/lib/apriltag)


set(LIB_NAME "corner_detector")
add_library(${LIB_NAME} SHARED)
target_sources(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/board_info.cpp
                                  ${CMAKE_CURRENT_LIST_DIR}/src/corner_point_detector.cpp)
target_compile_options(${LIB_NAME} PUBLIC -fPIC -Wall -O3)
set_target_properties(
  ${LIB_NAME}
  PROPERTIES CXX_STANDARD 14
             CXX_STANDARD_REQUIRED YES
             CXX_EXETENSIONS NO)
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(${LIB_NAME} SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/lib/apriltag/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${LIB_NAME} PUBLIC apriltag opencv_core opencv_highgui yaml-cpp)
