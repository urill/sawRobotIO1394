# - Config file for the Amp1394 package
# It defines the following variables
#  Amp1394_INCLUDE_DIR  - include directories for Amp1394
#  Amp1394_LIBRARY_DIR  - link library directories for Amp1394
#  Amp1394_LIBRARIES    - libraries to link against

# Version
set (Amp1394_VERSION_MAJOR "1")
set (Amp1394_VERSION_MINOR "3")
set (Amp1394_VERSION_PATCH "42")
set (Amp1394_VERSION "1.3.42")
 
# Compute paths
set (Amp1394_INCLUDE_DIR "/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib;/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394")
set (Amp1394_LIBRARY_DIR "/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib")
 
# Libraries to link against
set (Amp1394_LIBRARIES "Amp1394")

# FireWire/Ethernet support
set (Amp1394_HAS_RAW1394 "OFF")
set (Amp1394_HAS_PCAP    "OFF")
