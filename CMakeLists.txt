cmake_minimum_required(VERSION 3.5)

project(calibration LANGUAGES CXX)

# 设置自动MOC、RCC和UIC
set(CMAKE_AUTORCC ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

ament_package_xml()
set(ROS_DISTRO "ROS 2")
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")

add_subdirectory(${PROJECT_SOURCE_DIR}/lib/corner_detector)
add_subdirectory(${PROJECT_SOURCE_DIR}/lib/camera_model)


set(${PROJECT_NAME}_headers_to_moc
        include/rs_calibration.h
        include/calib_core.h
        ui/ui_calibration_widget.h
        )

foreach(header "${${PROJECT_NAME}_headers_to_moc}")
    qt5_wrap_cpp(${PROJECT_NAME}_moc_files "${header}")
endforeach()

set(${PROJECT_NAME}_source_files
        src/rs_calibration.cpp
        src/calib_core.cpp
        src/common_function.cpp
        src/lidar_calib.cpp
        src/camera_calib.cpp
        )

add_library(${PROJECT_NAME} SHARED
        ${${PROJECT_NAME}_moc_files}
        ${${PROJECT_NAME}_source_files}
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ui>
        $<INSTALL_INTERFACE:include>
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${cv_bridge_LIBRARIES}
        ${CERES_LIBRARIES}
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        Qt5::Widgets
        camera_model
        corner_detector
        )

ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
        pluginlib
        rclcpp
        rviz_assimp_vendor
        rviz_rendering
        rviz_common
        visualization_msgs
        PCL
        pcl_conversions
        image_transport
        cv_bridge
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
