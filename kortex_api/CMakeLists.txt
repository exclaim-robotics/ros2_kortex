cmake_minimum_required(VERSION 3.14)
include(FetchContent)
#include(ExternalProject)

# NOTE: the archive is selected and fetched *after* project(), because CMake
# does not populate CMAKE_SYSTEM_NAME / CMAKE_SYSTEM_PROCESSOR until then.
# Upstream sidestepped this by hardcoding the x86 URL up here.
project(kortex_api)

#Future:
# convert API_URL components from CMAKE_SYSTEM variables.
# The kinova artifactory server artifacts have slightly different expectations and they need some transformations
# Could build something like: (URL_OS)_(URL_PROCESSOR)_(?)_(URL_COMPILER).zip
# string(REPLACE "_" "-" URL_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR}) # x86_64 -> x86-64
# string(TOLOWER ${CMAKE_SYSTEM_NAME} URL_OS) # Linux -> linux

# Kinova publishes the binary API for aarch64 too; upstream just never wired it
# up, which is the only thing stopping kortex_driver building on a Jetson.
# Archives and hashes are from the same artifactory directory as the x86 one:
#   https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0/
set(KORTEX_API_BASE
  "https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.8.0")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(KORTEX_API_ARCHIVE "linux_x86-64_gcc_5.4.zip")
  set(KORTEX_API_MD5 "b9b68885df8e1dd0e5a2ba7285f14c0d")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  # gcc 5.4 build, matching the x86 choice. A gcc 7.4 build also exists
  # (md5 5c0998d099310f0cf7f9a4039cf1cafd) if an ABI problem ever shows up.
  set(KORTEX_API_ARCHIVE "linux_aarch64_gcc_5.4.zip")
  set(KORTEX_API_MD5 "062a42ac43a883e7aa48ef6ee53aca1f")
else()
  message(FATAL_ERROR
    "No Kortex binary API for ${CMAKE_SYSTEM_NAME}/${CMAKE_SYSTEM_PROCESSOR}. "
    "Supported: Linux x86_64, Linux aarch64.")
endif()

set(API_URL "${KORTEX_API_BASE}/${KORTEX_API_ARCHIVE}")

FetchContent_Declare(
  kinova_binary_api
  URL "${API_URL}"
  URL_HASH MD5=${KORTEX_API_MD5}
)
FetchContent_MakeAvailable(kinova_binary_api)

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
