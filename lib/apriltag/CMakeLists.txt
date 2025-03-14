cmake_minimum_required(VERSION 3.1)
project(
  apriltag
  VERSION 3.3.0
  LANGUAGES C CXX)

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()
option(BUILD_APRILTAG_SHARED_LIBS "Build shared libraries" ON)
option(ASAN "Use AddressSanitizer for debug builds to detect memory issues" OFF)
option(BUILD_APRILTAG_EXAMPLE "Build example or not" OFF)

if(ASAN)
  set(ASAN_FLAGS
      "\
        -fsanitize=address \
        -fsanitize=bool \
        -fsanitize=bounds \
        -fsanitize=enum \
        -fsanitize=float-cast-overflow \
        -fsanitize=float-divide-by-zero \
        -fsanitize=nonnull-attribute \
        -fsanitize=returns-nonnull-attribute \
        -fsanitize=signed-integer-overflow \
        -fsanitize=undefined \
        -fsanitize=vla-bound \
        -fno-sanitize=alignment \
        -fsanitize=leak \
        -fsanitize=object-size \
    ")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
endif()

# Set a default build type if none was specified
set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()

if(${BUILD_APRILTAG_SHARED_LIBS})
  add_library(${PROJECT_NAME} SHARED)
else()
  add_library(${PROJECT_NAME} STATIC)
endif(${BUILD_APRILTAG_SHARED_LIBS})

aux_source_directory(src/common APRILTAG_COMMON_SRC)
aux_source_directory(src/tag APRILTAG_TAG_SRC)
target_sources(
  ${PROJECT_NAME}
  PRIVATE ${APRILTAG_COMMON_SRC} ${APRILTAG_TAG_SRC} src/apriltag.c
          src/apriltag_pose.c src/apriltag_quad_thresh.c)

if(MSVC)
  add_compile_definitions("_CRT_SECURE_NO_WARNINGS")
else()
  find_package(Threads REQUIRED)
  target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads m)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION 3 VERSION
                                                             ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")

include(GNUInstallDirs)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include/apriltag)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>/apriltag")

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