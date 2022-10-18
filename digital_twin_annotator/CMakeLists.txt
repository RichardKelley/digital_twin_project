cmake_minimum_required(VERSION 3.16)

project(dt_annotator
  VERSION 0.1
  DESCRIPTION "Annotator for Digital Twin Program"
  LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++2a")

include(CheckLanguage)
check_language(CUDA)
if (CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
else()
  message(STATUS "No cuda compiler detected.")
endif()

set(CMAKE_PREFIX_PATH "$ENV{HOME}/opt/libtorch")
find_package(Torch REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(SDL2 REQUIRED)
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)

add_executable(dt_annotator
  src/main.cpp
  src/render_system.cpp
  src/input_system.cpp
  src/map_tile_mesh.cpp
  )
target_include_directories(dt_annotator PUBLIC
  ${FMT_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${GLEW_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )
target_link_directories(dt_annotator PUBLIC
  /usr/local/lib64
  ${PCL_LIBRARY_DIRS}
  )
target_link_libraries(dt_annotator
  stdc++fs
  fmt::fmt
  ${TORCH_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)


