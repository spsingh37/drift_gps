# Install script for directory: /home/neofelis/VRX/drift

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/neofelis/VRX/drift/build/tests/cmake_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libdrift.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/neofelis/VRX/drift/build/libdrift.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/estimator" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/estimator/inekf_estimator.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/filter" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/filter/base_correction.h"
    "/home/neofelis/VRX/drift/include/drift/filter/base_propagation.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/filter/inekf" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/filter/inekf/inekf.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/filter/inekf/correction" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/filter/inekf/correction/legged_kinematics_correction.h"
    "/home/neofelis/VRX/drift/include/drift/filter/inekf/correction/pose_correction.h"
    "/home/neofelis/VRX/drift/include/drift/filter/inekf/correction/velocity_correction.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/filter/inekf/propagation" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/filter/inekf/propagation/imu_propagation.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/math" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/math/lie_group.h"
    "/home/neofelis/VRX/drift/include/drift/math/se_k_3.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/state" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/state/robot_state.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/imu_filter" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/imu_filter/imu_ang_vel_ekf.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/kinematics" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/kinematics/mini_cheetah_kinematics.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/measurement" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/measurement/angular_velocity.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/contact.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/imu.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/joint_state.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/legged_kinematics.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/measurement.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/navsat.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/odom.h"
    "/home/neofelis/VRX/drift/include/drift/measurement/velocity.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/measurement/impl" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/angular_velocity_impl.cpp"
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/contact_impl.cpp"
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/imu_impl.cpp"
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/joint_state_impl.cpp"
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/navsat_impl.cpp"
    "/home/neofelis/VRX/drift/include/drift/measurement/impl/velocity_impl.cpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/kinematics/robots/mini_cheetah" TYPE FILE FILES
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/H_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/H_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/H_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/H_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jb_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jb_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jb_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jb_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jp_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jp_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Js_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Js_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Js_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Js_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvb_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvb_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvb_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvb_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvs_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvs_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvs_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jvs_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jwb_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jwb_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jwb_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jwb_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jws_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jws_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jws_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/Jws_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/R_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/R_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/R_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/R_Body_to_HindRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/p_Body_to_FrontLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/p_Body_to_FrontRightFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/p_Body_to_HindLeftFoot.h"
    "/home/neofelis/VRX/drift/include/drift/kinematics/robots/mini_cheetah/p_Body_to_HindRightFoot.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/drift/utils" TYPE FILE FILES "/home/neofelis/VRX/drift/include/drift/utils/type_def.h")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/drift/driftTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/drift/driftTargets.cmake"
         "/home/neofelis/VRX/drift/build/CMakeFiles/Export/lib/cmake/drift/driftTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/drift/driftTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/drift/driftTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/drift" TYPE FILE FILES "/home/neofelis/VRX/drift/build/CMakeFiles/Export/lib/cmake/drift/driftTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/drift" TYPE FILE FILES "/home/neofelis/VRX/drift/build/CMakeFiles/Export/lib/cmake/drift/driftTargets-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/drift" TYPE FILE FILES
    "/home/neofelis/VRX/drift/build/driftConfig.cmake"
    "/home/neofelis/VRX/drift/build/driftConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/neofelis/VRX/drift/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
