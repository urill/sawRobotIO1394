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

# Example program uses curses for text-based display
find_package (Curses)
if (CURSES_FOUND)
  add_subdirectory(curses)
else (CURSES_FOUND)
  message ("sawRobotIO1394 curses based example not available -- curses lib not found")
endif (CURSES_FOUND)

# Qt based example, CMake will look for cisstQt
add_subdirectory (Qt)
