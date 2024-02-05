cmake_minimum_required(VERSION 3.24)

set(3DPARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3dparty CACHE INTERNAL "Location of third-party dependencies")

set(
  CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  ${3DPARTY_DIR}/embedded-cmake-snippets
  ${CMAKE_MODULE_PATH}
)



find_package(ArmToolchain REQUIRED)
find_package(MCU  REQUIRED)


project(nucleo431-experiments LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_subdirectory(drivers)
add_subdirectory(src)


