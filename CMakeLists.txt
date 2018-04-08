cmake_minimum_required(VERSION 3.0)
project(ppf_foropen3d)

set(CMAKE_CXX_STANDARD 11)


file(GLOB_RECURSE HEADER_FILES "./include/*.h")
file(GLOB_RECURSE SOURCE_FILES "./src/*.cpp")

include_directories(include)
include_directories(/usr/include/eigen3/)

#add_executable(surface_matching
#        ${HEADER_FILES} ${SOURCE_FILES}
#        )

add_executable(test_1
        main.cpp
        src/Pose3D.cpp
        include/c_utils.h
        include/hash_murmur.h
        include/hash_murmur64.h
        include/Pose3D.h
        include/THashInt.h
        src/THashInt.cpp
        include/PPFHelpers.h
        src/PPFHelper.cpp
#        include/Precomp.h
        )

add_library(test_helper
        include/c_utils.h
        include/PPFHelpers.h
        src/PPFHelper.cpp)

add_executable(test_typedef test_typedef.cpp)

add_executable(test_for_eigen main_for_eigen.cpp)

add_executable(test_flann test_flann.cpp include/nanoflann.h include/utils.h)

add_executable(test_flann_eigen test_flann_eigen.cpp)

add_executable(test_ppfhelper test_ppfhelper.cpp)
target_link_libraries(test_ppfhelper test_helper)


