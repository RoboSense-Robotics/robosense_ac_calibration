cmake_minimum_required(VERSION 3.5)
project(camera_model)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d highgui)

# 定义库
add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} 
  PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/src/camera_model_factory.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pinhole.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/fisheye.cpp
)
target_compile_options(${PROJECT_NAME} PUBLIC -fPIC -Wall -O3)

# 设置属性
set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES 
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# 包含路径
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

# 链接库
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS})
