# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "diff_drive_robot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idiff_drive_robot:/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg;-Istd_msgs:/usr/local/MATLAB/R2023a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(diff_drive_robot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg" NAME_WE)
add_custom_target(_diff_drive_robot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "diff_drive_robot" "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(diff_drive_robot
  "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diff_drive_robot
)

### Generating Services

### Generating Module File
_generate_module_cpp(diff_drive_robot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diff_drive_robot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(diff_drive_robot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(diff_drive_robot_generate_messages diff_drive_robot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg" NAME_WE)
add_dependencies(diff_drive_robot_generate_messages_cpp _diff_drive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diff_drive_robot_gencpp)
add_dependencies(diff_drive_robot_gencpp diff_drive_robot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diff_drive_robot_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(diff_drive_robot
  "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diff_drive_robot
)

### Generating Services

### Generating Module File
_generate_module_py(diff_drive_robot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diff_drive_robot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(diff_drive_robot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(diff_drive_robot_generate_messages diff_drive_robot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg" NAME_WE)
add_dependencies(diff_drive_robot_generate_messages_py _diff_drive_robot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(diff_drive_robot_genpy)
add_dependencies(diff_drive_robot_genpy diff_drive_robot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS diff_drive_robot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diff_drive_robot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/diff_drive_robot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(diff_drive_robot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diff_drive_robot)
  install(CODE "execute_process(COMMAND \"/home/jorand/.matlab/R2023a/ros1/glnxa64/venv/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diff_drive_robot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/diff_drive_robot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(diff_drive_robot_generate_messages_py std_msgs_generate_messages_py)
endif()
