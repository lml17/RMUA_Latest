project(roborts_decision)
cmake_minimum_required(VERSION 3.1)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
set(Protobuf_PREFIX_PATH
"/usr/local/protobuf/include"
"/usr/local/protobuf/lib"
"/usr/local/protobuf/bin"
)
list(APPEND CMAKE_PREFIX_PATH "${Protobuf_PREFIX_PATH}")
find_package(ProtoBuf REQUIRED PATH /usr/local/protobuf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  )


find_package(Eigen3 REQUIRED)
#find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(

)





#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )


catkin_package()


# decision_node
add_executable(decision_node
  decision_node.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  executor/chassis_executor.cpp
  )

target_link_libraries(decision_node 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )
# sel_behavior_node
add_executable(sel_behavior_node
  sel_behavior_node.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/defend_behavior.h
  example_behavior/back_behavior.h
  example_behavior/TurnToDetectedDirection.h
  example_behavior/gimbal_behavior.h
  example_behavior/gimbal_rate_behavior.h
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  )

target_link_libraries(sel_behavior_node 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )



# blue_master
add_executable(blue_master
  blue_master.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  example_behavior/swing_behavior.h
  example_behavior/shoot_behavior.h
  example_behavior/defend_behavior.h
  example_behavior/back_behavior.h
  example_behavior/TurnToDetectedDirection.h
  example_behavior/gimbal_behavior.h
  example_behavior/gimbal_rate_behavior.h
  executor/chassis_executor.cpp
  executor/gimbal_executor.cpp
  example_behavior/froze_behavior.h
  )

target_link_libraries(blue_master 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )


# blue_wing
add_executable(blue_wing
  blue_wing.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  executor/chassis_executor.cpp
  )

target_link_libraries(blue_wing 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )




# red_master
add_executable(red_master
  red_master.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  executor/chassis_executor.cpp
  )

target_link_libraries(red_master 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )


# red_wing
add_executable(red_wing
  red_wing.cpp
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  example_behavior/escape_behavior.h
  example_behavior/back_boot_area_behavior.h
  example_behavior/chase_behavior.h
  example_behavior/patrol_behavior.h
  example_behavior/search_behavior.h
  executor/chassis_executor.cpp
  )

target_link_libraries(red_wing 
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  #${PROTOBUF_LIBRARIES}
  -lzmq
  )

# 键盘输入
add_executable(
  keyboard_node keyboard_node.cpp
)
add_dependencies(keyboard_node roborts_msgs_generate_messages_cpp)
target_link_libraries(
  keyboard_node ${catkin_LIBRARIES}
)
catkin_install_python(PROGRAMS teleop_twist_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



