cmake_minimum_required(VERSION 3.10)
project(ppf_foropen3d)

set(CMAKE_CXX_STANDARD 11)


file(GLOB_RECURSE HEADER_FILES "./include/*.h")
file(GLOB_RECURSE SOURCE_FILES "./src/*.cpp")

include_directories(include)
include_directories(/usr/include/eigen3/)

add_executable(surface_matching
        ${HEADER_FILES} ${SOURCE_FILES}
        )

