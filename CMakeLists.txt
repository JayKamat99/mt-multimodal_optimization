cmake_minimum_required(VERSION 3.16)
project(mt_multimodalOptimization)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-O3 -g -Wall -fPIC -std=c++14 -Wno-terminate)

include_directories(ompl/src)

add_subdirectory(ompl)

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}/rai\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG
  -DRAI_SWIFT -DRAI_GJK -DRAI_FCL -DRAI_CCD
  -DRAI_BIND_KOMO
  )

include_directories(
  /usr/include/eigen3
  rai/rai
  )

file(GLOB SRC_Core rai/rai/Core/*.cpp)
file(GLOB SRC_Optim rai/rai/Optim/*.cpp)
file(GLOB SRC_Geo rai/rai/Geo/*.cpp)
file(GLOB SRC_GeoPlus rai/rai/Geo/GJK/*.c)
file(GLOB SRC_Algo rai/rai/Algo/*.cpp)
file(GLOB SRC_Plot rai/rai/Plot/*.cpp)
file(GLOB SRC_Gui rai/rai/Gui/*.cpp)
file(GLOB SRC_Kin rai/rai/Kin/*.cpp)
file(GLOB SRC_KinSwift rai/rai/Kin/SWIFT/*.cpp)
file(GLOB SRC_KOMO rai/rai/KOMO/*.cpp)
# file(GLOB SRC_Logic rai/rai/Logic/*.cpp)
# file(GLOB SRC_LGP rai/rai/LGP/*.cpp)
# file(GLOB SRC_RosCom rai/rai/RosCom/*.cpp)
file(GLOB SRC_Perception rai/rai/Perception/*.cpp)
file(GLOB SRC_Control rai/rai/Control/*.cpp)
# file(GLOB SRC_Operate rai/rai/Operate/*.cpp)
# file(GLOB SRC_ry rai/rai/ry/*.cpp)

add_library(rai SHARED
  ${SRC_Core}
  ${SRC_Optim}
  ${SRC_Geo}
  ${SRC_GeoPlus}
  ${SRC_Algo}
  ${SRC_Plot}
  ${SRC_Gui}
  ${SRC_Kin}
  ${SRC_KinSwift}
  ${SRC_KOMO}
  #  ${SRC_Logic}
  #  ${SRC_LGP}
  #  ${SRC_RosCom}
  ${SRC_Perception}
  ${SRC_Control}
  #  ${SRC_Operate}
  #  ${SRC_ry}
  )


# try to add one 





# file(GLOB MM_OPT src/multimodal_optimizers/src/*.cpp)

# # file(GLOB ML ./src/multilevel/datastructures/src/*.cpp
# # ./src/multilevel/datastructures/src/*.cpp)

# file(GLOB_RECURSE SRC_ML src/multilevel/planners/*.cpp)


# message(STATUS "SRC_ML: ${SRC_ML}")







# # target_link_libraries(JAY rai ompl)

target_link_libraries(rai
  jsoncpp lapack blas pthread rt X11
  assimp fcl ann glfw GLEW glut GLU GL png qhull
  )

# # add_executable(mt_multimodalOptimization
# #         src/main.cpp
# #         src/Optimizer.h src/Optimizer.cpp)

# add_executable(mt_multimodalOptimization
#         src/main.cpp
#         src/Optimizer.cpp
#         src/multimodal_optimizers/src/PathOptimizer.cpp
#         src/multimodal_optimizers/src/PathOptimizerKOMO.cpp)



# # target_link_libraries(mt_multimodalOptimization JAY rai ompl)
# target_link_libraries(mt_multimodalOptimization rai ompl)

# add_executable(trial
#           src/mainRai.cpp)

# target_link_libraries(trial rai ompl)


include_directories(src)

file(GLOB SRC_PATH src/path/*.cpp)
add_library(Path SHARED ${SRC_PATH} )
target_link_libraries(Path rai ompl)

# executable that only depends on ompl
add_executable(t_ompl examples/ompl/main.cpp)
target_link_libraries( t_ompl ompl )

# executable thatdepends on ompl and rai
add_executable(t_ompl_komo ./examples/ompl-komo/main.cpp)
target_link_libraries( t_ompl_komo ompl rai )

# executable that uses all
add_executable(t_manip ./examples/Manipulator/main.cpp)
target_link_libraries( t_manip ompl rai Path )

# Example executable
add_executable(t_example ./examples/Just_Visualize_Example/main.cpp)
target_link_libraries( t_example rai )

# Example executable
add_executable(t_komo ./examples/only_KOMO/main.cpp)
target_link_libraries( t_komo rai )

# Main executable
add_executable(t_main ./examples/Main/main.cpp)
target_link_libraries( t_main ompl rai Path)

# try_path executable
add_executable(t_path ./examples/Test_KOMO/main.cpp)
target_link_libraries( t_path rai ompl Path )

# sequentialPlanning executable
add_executable(t_sequential ./examples/SequentialPlanning/main.cpp)
target_link_libraries( t_sequential rai ompl Path )