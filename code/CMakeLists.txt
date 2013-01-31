 #
# $Id: CMakeLists.txt 2593 2011-06-02 22:41:14 zchen24 $
#
# (C) Copyright 2012-2013 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---


################### sawFPGA1394QLALib ########################
 
set( REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask 
     cisstParameterTypes )


find_package(cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if(cisst_FOUND_AS_REQUIRED)

  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

  # Allow generated c++ code to find header files
  include_directories(${CMAKE_CURRENT_SOURCE_DIR})

  # To link with build libraries
  link_directories(${CMAKE_CURRENT_BINARY_DIR})

  set (sawRobotIO1394_INCLUDE_DIR "${sawRobotIO1394_SOURCE_DIR}/include")
  include_directories(${sawRobotIO1394_INCLUDE_DIR})
  set (sawRobotIO1394_HEADER_DIR "${sawRobotIO1394_INCLUDE_DIR}/sawRobotIO1394")

  # create the library
  add_library( sawRobotIO1394 mtsRobotIO1394.cpp
                              ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394.h
                              RobotInternal.cpp
                              RobotInternal.h )

  target_link_libraries(sawRobotIO1394 AmpIO)

  # link rtai lib (may need to add Xenomai support)
  if(CISST_HAS_LINUX_RTAI)
    target_link_libraries(sawRobotIO1394 ${RTAI_LIBRARIES})
  endif(CISST_HAS_LINUX_RTAI)

  # link cisst lib
  cisst_target_link_libraries(sawRobotIO1394 ${REQUIRED_CISST_LIBRARIES} )

endif(cisst_FOUND_AS_REQUIRED)









                                                        
