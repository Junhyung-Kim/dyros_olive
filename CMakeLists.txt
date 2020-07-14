project(testbed_jjs)
cmake_minimum_required(VERSION 2.8.4)
add_compile_options(-std=c++11)

find_package(Curses REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  ~/eigen3
  /usr/local/include/soem
  ${CMAKE_SOURCE_DIR}/include
)

add_library(dyros_red_ethercat red_ec_master.cpp)
add_library(controller controller.cpp)
add_library(dyros_red_model dyros_red_model.cpp)
add_library(sensoray826 sensoray826.cpp)

add_executable(${PROJECT_NAME} ./main.cpp)

target_link_libraries(${PROJECT_NAME}
  dyros_red_ethercat
  dyros_red_model
  sensoray826
  controller
  -lsoem
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib826_64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libOMD.so.1.5.1
  #-l826_64
  -lboost_system
  Boost::thread
  #-lOMD
)
