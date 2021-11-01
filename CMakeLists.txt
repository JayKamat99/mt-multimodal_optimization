cmake_minimum_required(VERSION 3.19)
project(mt_multimodalOptimization)
set(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(ompl 1.5.2 REQUIRED)
MESSAGE(${OMPL_VERSION})
MESSAGE(${OMPL_INCLUDE_DIRS})
MESSAGE(${OMPL_LIBRARIES})

list(APPEND INCLUDE_DIRS ${OMPL_INCLUDE_DIRS})
list(APPEND LIBRARIES ${OMPL_LIBRARIES})

include_directories(${INCLUDE_DIRS})
include_directories(rai/rai/)

link_libraries(${LIBRARIES})
link_libraries(rai/lib/libCore.so
rai/lib/libAlgo.so
rai/lib/libKin.so
rai/lib/libGui.so
rai/lib/libGeo.so
rai/lib/libKOMO.so
rai/lib/libOptim.so)

add_executable(mt_multimodalOptimization
        src/main.cpp
        src/Optimizer.h src/Optimizer.cpp)