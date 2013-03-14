#
# $Id$
#
# (C) Copyright 2013 Johns Hopkins University (JHU), All Rights
# Reserved.
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
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  qt4_wrap_cpp (SAW_ROBOTIO1394_QT_WRAP_CPP
                ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394QtWidget.h)
  add_library (sawRobotIO1394Qt
               ${sawRobotIO1394_HEADER_DIR}/mtsRobotIO1394QtWidget.h
               mtsRobotIO1394QtWidget.cpp
               ${SAW_ROBOTIO1394_QT_WRAP_CPP})
  set_property (TARGET sawRobotIO1394Qt PROPERTY FOLDER "sawRobotIO1394")
  cisst_target_link_libraries (sawRobotIO1394Qt ${REQUIRED_CISST_LIBRARIES})

  # make sure the new library is known by the parent folder to add to the config file
  set (sawRobotIO1394Qt_LIBRARIES sawRobotIO1394Qt PARENT_SCOPE)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
