#
# (C) Copyright 2012-2014 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawRobotIO1394)

add_subdirectory (code)

# Examples of config files
add_subdirectory (share)

# Examples
add_subdirectory (examples)

# Tests programs, will only get compiled if cisstTestsDriver has been compiled
add_subdirectory (tests)
