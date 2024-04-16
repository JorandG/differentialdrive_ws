# Install script for directory: /home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_drive_robot/msg" TYPE FILE FILES
    "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/HumanRobotInteraction.msg"
    "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/MILPResult.msg"
    "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/msg/TimeMessage.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_drive_robot/cmake" TYPE FILE FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/build/diff_drive_robot/catkin_generated/installspace/diff_drive_robot-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/devel/include/diff_drive_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/jorand/.matlab/R2023a/ros1/glnxa64/venv/bin/python3" -m compileall "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/devel/lib/python3/dist-packages/diff_drive_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/devel/lib/python3/dist-packages/diff_drive_robot")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/build/diff_drive_robot/catkin_generated/installspace/diff_drive_robot.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_drive_robot/cmake" TYPE FILE FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/build/diff_drive_robot/catkin_generated/installspace/diff_drive_robot-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_drive_robot/cmake" TYPE FILE FILES
    "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/build/diff_drive_robot/catkin_generated/installspace/diff_drive_robotConfig.cmake"
    "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/build/diff_drive_robot/catkin_generated/installspace/diff_drive_robotConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_drive_robot" TYPE FILE FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/devel/lib/libdiff_drive_robot_matlab.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdiff_drive_robot_matlab.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdiff_drive_robot_matlab.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libdiff_drive_robot_matlab.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "/home/jorand/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/src/diff_drive_robot/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

