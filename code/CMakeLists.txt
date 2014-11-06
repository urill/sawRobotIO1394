#
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

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # Compile lib Amp1394 and examples within this project, this relies on svn:externals
  add_subdirectory (Amp1394)
  include_directories ("${CMAKE_CURRENT_SOURCE_DIR}/Amp1394/lib")

  # Modify CMake configuration to use cisst
  include (${CISST_USE_FILE})

  # Allow c++ code to find local header files
  include_directories (${CMAKE_CURRENT_SOURCE_DIR})

  # create/configure file for find_package (sawRobotIO1394)
  set (sawRobotIO1394_INCLUDE_DIR "${sawRobotIO1394_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/Amp1394/lib")
  set (sawRobotIO1394_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawRobotIO1394_LIBRARIES sawRobotIO1394 Amp1394 raw1394)

  include_directories (${sawRobotIO1394_INCLUDE_DIR})
  set (sawRobotIO1394_HEADER_DIR "${sawRobotIO1394_SOURCE_DIR}/include/sawRobotIO1394")

  # create the library
  add_library (sawRobotIO1394
               ${sawRobotIO1394_HEADER_DIR}/osaConfiguration1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaPort1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaRobot1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaDigitalInput1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaDigitalOutput1394.h
               ${sawRobotIO1394_HEADER_DIR}/osaXML1394.h
               osaRobot1394.cpp
               osaDigitalInput1394.cpp
               osaDigitalOutput1394.cpp
               osaPort1394.cpp
               osaXML1394.cpp
               ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394.h
               mtsRobot1394.h
               mtsRobot1394.cpp
               mtsDigitalInput1394.h
               mtsDigitalInput1394.cpp
               mtsDigitalOutput1394.h
               mtsDigitalOutput1394.cpp
               mtsRobotIO1394.cpp)

  set_property (TARGET sawRobotIO1394 PROPERTY FOLDER "sawRobotIO1394")
  target_link_libraries (sawRobotIO1394 Amp1394)

  # link rtai lib (may need to add Xenomai support)
  if (CISST_HAS_LINUX_RTAI)
    target_link_libraries (sawRobotIO1394 ${RTAI_LIBRARIES})
  endif (CISST_HAS_LINUX_RTAI)

  # link cisst lib
  cisst_target_link_libraries (sawRobotIO1394 ${REQUIRED_CISST_LIBRARIES})

  # add Qt code
  add_subdirectory (Qt)
  set (sawRobotIO1394_LIBRARIES ${sawRobotIO1394_LIBRARIES} ${sawRobotIO1394Qt_LIBRARIES})

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawRobotIO1394Config.cmake.in"
                  "${sawRobotIO1394_BINARY_DIR}/sawRobotIO1394Config.cmake"
                   @ONLY@)

endif(cisst_FOUND_AS_REQUIRED)
