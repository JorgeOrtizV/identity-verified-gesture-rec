# Install script for directory: /home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jorge/Documents/ros_espros/cam660_apps/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tof_preprocessing/msg" TYPE FILE FILES
    "/home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing/msg/Blob.msg"
    "/home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing/msg/BlobArray.msg"
    "/home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing/msg/AgentMsg.msg"
    "/home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing/msg/AgentArray.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tof_preprocessing/cmake" TYPE FILE FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tof_preprocessing-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/jorge/Documents/ros_espros/cam660_apps/devel/include/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/jorge/Documents/ros_espros/cam660_apps/devel/share/roseus/ros/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/jorge/Documents/ros_espros/cam660_apps/devel/share/common-lisp/ros/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/jorge/Documents/ros_espros/cam660_apps/devel/share/gennodejs/ros/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/jorge/Documents/ros_espros/cam660_apps/devel/lib/python3/dist-packages/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/jorge/Documents/ros_espros/cam660_apps/devel/lib/python3/dist-packages/tof_preprocessing")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tof_preprocessing.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tof_preprocessing/cmake" TYPE FILE FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tof_preprocessing-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tof_preprocessing/cmake" TYPE FILE FILES
    "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tof_preprocessingConfig.cmake"
    "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tof_preprocessingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tof_preprocessing" TYPE FILE FILES "/home/jorge/Documents/ros_espros/cam660_apps/src/tof_preprocessing/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/depth_subscriber.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/preprocessing.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/vis_fg_mask.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/vis_depth_clean.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/vis_blobs.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tracking.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/tracking2.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tof_preprocessing" TYPE PROGRAM FILES "/home/jorge/Documents/ros_espros/cam660_apps/build/tof_preprocessing/catkin_generated/installspace/vis_kalman.py")
endif()

