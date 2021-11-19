cmake_minimum_required(VERSION 3.0.0)
project(opencvtest VERSION 0.1.0)

set(CMAKE_TOOLCHAIN_FILE "C:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/dev/vcpkg/installed/x64-windows/share")
set(OpenCV_DIR "C:/dev/vcpkg/installed/x64-windows/share/opencv")

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})

add_library(
    CVUI
    cvui.h
    cvui.cpp
)

add_library(
    funcoes
    Funcs.h
    Funcs.cpp
)

add_executable(opencvtest main.cpp)
link_directories(${OpenCV_LIB_DIR})
target_link_libraries( opencvtest PRIVATE ${OpenCV_LIBS}  funcoes CVUI  )