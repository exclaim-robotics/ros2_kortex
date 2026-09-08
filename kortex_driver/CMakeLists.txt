cmake_minimum_required(VERSION 3.14)
include(FetchContent)

# NOTE: the archive is selected and fetched AFTER project(), because CMake does
# not populate CMAKE_SYSTEM_NAME / CMAKE_SYSTEM_PROCESSOR until then. Upstream
# sidestepped that by hardcoding the x86 URL up here, which is also why the
# arch check further down could only ever fail.
project(kortex_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# this is needed by the kortex_api to tell it we are compiling for linux
add_definitions(-D_OS_UNIX=1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kortex_api REQUIRED)

# This is the second, independent arch gate in ros2_kortex - kortex_api has
# its own - and it pins a DIFFERENT API version, 2.5.0 against kortex_api's
# 2.8.0. That matters here because Kinova never published an aarch64 build of
# 2.5.0: that release has x86-64 and 32-bit ARM only. 2.8.0 does have one, and
# kortex_api already fetches 2.8.0, so on aarch64 we take the static library
# from 2.8.0 as well. The aarch64 build is therefore MORE self-consistent than
# the x86 one, which links a 2.5.0 library against 2.8.0 headers.
#
# x86_64 is left exactly as upstream ships it. It is known-good and churning
# it to 2.8.0 would be a real behavioural change for no reason we have.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.5.0/linux_x86-64_x86_gcc.zip")
  set(API_MD5 "64bd86e7ab8bda90ef1fc7d6a356e080")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(API_URL "https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.8.0/linux_aarch64_gcc_5.4.zip")
  set(API_MD5 "062a42ac43a883e7aa48ef6ee53aca1f")
else()
  message(FATAL_ERROR
    "No Kortex binary API for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. "
    "Supported: Linux x86_64, Linux aarch64.")
endif()

FetchContent_Declare(
  kinova_binary_api
  URL "${API_URL}"
  URL_HASH MD5=${API_MD5}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(kinova_binary_api)

# CMake does not allow IMPORTED libraries to be installed
# The package kortex_api will download and setup the include directories
add_library(KortexApiCpp STATIC IMPORTED)
set_target_properties(KortexApiCpp PROPERTIES
  IMPORTED_LOCATION ${kinova_binary_api_SOURCE_DIR}/lib/release/libKortexApiCpp.a
  INTERFACE_LINK_LIBRARIES KortexApiCpp
)
target_link_libraries(KortexApiCpp INTERFACE pthread)
add_dependencies(KortexApiCpp kortex_api)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/kortex_math_util.cpp
)
target_link_libraries(${PROJECT_NAME} KortexApiCpp)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
# kortex_api is the headers for the Kortex API
ament_target_dependencies(
  ${PROJECT_NAME}
  SYSTEM kortex_api
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  kortex_api
  pluginlib
  rclcpp
)
ament_package()
