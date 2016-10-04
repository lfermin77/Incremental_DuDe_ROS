cmake_minimum_required(VERSION 2.8.3)
project(inc_dude)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


find_package(CGAL REQUIRED COMPONENTS Core)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(MPFR)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dude_ros
#  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final  
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src  
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/mathtool
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw
  ${CGAL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  
  ${PROJECT_SOURCE_DIR}/FIST  
)
include(${CGAL_USE_FILE})




######## Files from DuDe

file(GLOB DuDe_SOURCES
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/*.cpp
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton/*.cpp
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/graph/*.cpp
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw/*.cpp

)









######## inc_DuDe_ros
add_executable(inc_dude src/inc_dude.cpp  include/wrapper.cpp include/inc_decomp.cpp  ${DuDe_SOURCES})

target_link_libraries(inc_dude
   ${catkin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/FIST.lib
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/libFIST.linux64.a
   ${CGAL_LIBRARIES} 
   ${CGAL_3RD_PARTY_LIBRARIES}
   ${MPFR_LIBRARIES}
 )


add_dependencies(inc_dude ${catkin_LIBRARIES} ${CGAL_LIBRARIES})



######## evaluation
add_executable(evaluation src/evaluation.cpp  include/wrapper.cpp include/inc_decomp.cpp  ${DuDe_SOURCES})

target_link_libraries(evaluation
   ${catkin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/FIST.lib
   ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/libFIST.linux64.a
   ${CGAL_LIBRARIES} 
   ${CGAL_3RD_PARTY_LIBRARIES}
   ${MPFR_LIBRARIES}
 )


add_dependencies(evaluation ${catkin_LIBRARIES} ${CGAL_LIBRARIES})










