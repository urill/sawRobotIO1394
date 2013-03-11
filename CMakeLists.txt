#
# $Id$
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

project(sawRobotIO1394)

add_subdirectory(code)
file(COPY share DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
# Example program uses curses for text-based display
find_package (Curses)
if (CURSES_FOUND)
  add_subdirectory(examples)
else (CURSES_FOUND)
  message ("sawRobotIO1394 examples not available -- curses lib not found")
endif (CURSES_FOUND)

# Tests programs, will only get compiled if cisstTestsDriver has been compiled
add_subdirectory (tests)
