cmake_minimum_required(VERSION 3.14)
include(FetchContent)
#include(ExternalProject)
FetchContent_Declare(
  kinova_binary_api
  URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.8.0/linux_aarch64_gcc_7.4.zip
  URL_HASH MD5=5c0998d099310f0cf7f9a4039cf1cafd
)

FetchContent_MakeAvailable(kinova_binary_api)

project(kortex_api)

#Future:
# convert API_URL components from CMAKE_SYSTEM variables.
# The kinova artifactory server artifacts have slightly different expectations and they need some transformations
# Could build something like: (URL_OS)_(URL_PROCESSOR)_(?)_(URL_COMPILER).zip
# string(REPLACE "_" "-" URL_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}) # x86_64 -> x86-64
# string(TOLOWER ${CMAKE_SYSTEM_NAME} URL_OS) # Linux -> linux

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(API_URL https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.8.0/linux_aarch64_gcc_7.4.zip)
else()
  set(API_URL "")
  message(FATAL_ERROR "Unsupported System: currently support is for Linux aarch64. Detected ${CMAKE_SYSTEM_NAME} and ${CMAKE_SYSTEM_PROCESSOR}")
endif()

#ExternalProject_Add(kinova_binary_api
#  URL ${API_URL}
#  CONFIGURE_COMMAND ""
#  BUILD_COMMAND ""
#  INSTALL_COMMAND ""
#)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include/kortex_api)

install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/common/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/messages/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/client_stubs/ DESTINATION include/kortex_api)
install(DIRECTORY ${kinova_binary_api_SOURCE_DIR}/include/google/protobuf DESTINATION include/kortex_api/google)

ament_package()
