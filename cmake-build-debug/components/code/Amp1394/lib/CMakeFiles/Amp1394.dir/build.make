# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/cos/bin/clion-2017.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/cos/bin/clion-2017.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug

# Include any dependencies generated for this target.
include components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/depend.make

# Include the progress variables for this target.
include components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/progress.make

# Include the compile flags for this target's objects.
include components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/flags.make

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/flags.make
components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o: ../components/code/Amp1394/lib/code/AmpIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o -c /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/AmpIO.cpp

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Amp1394.dir/code/AmpIO.cpp.i"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/AmpIO.cpp > CMakeFiles/Amp1394.dir/code/AmpIO.cpp.i

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Amp1394.dir/code/AmpIO.cpp.s"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/AmpIO.cpp -o CMakeFiles/Amp1394.dir/code/AmpIO.cpp.s

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.requires:

.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.requires

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.provides: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.requires
	$(MAKE) -f components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/build.make components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.provides.build
.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.provides

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.provides.build: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o


components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/flags.make
components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o: ../components/code/Amp1394/lib/code/Amp1394Time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o -c /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/Amp1394Time.cpp

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.i"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/Amp1394Time.cpp > CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.i

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.s"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib/code/Amp1394Time.cpp -o CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.s

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.requires:

.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.requires

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.provides: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.requires
	$(MAKE) -f components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/build.make components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.provides.build
.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.provides

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.provides.build: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o


# Object files for target Amp1394
Amp1394_OBJECTS = \
"CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o" \
"CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o"

# External object files for target Amp1394
Amp1394_EXTERNAL_OBJECTS =

components/code/Amp1394/lib/libAmp1394.a: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o
components/code/Amp1394/lib/libAmp1394.a: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o
components/code/Amp1394/lib/libAmp1394.a: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/build.make
components/code/Amp1394/lib/libAmp1394.a: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libAmp1394.a"
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && $(CMAKE_COMMAND) -P CMakeFiles/Amp1394.dir/cmake_clean_target.cmake
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Amp1394.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/build: components/code/Amp1394/lib/libAmp1394.a

.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/build

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/requires: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/AmpIO.cpp.o.requires
components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/requires: components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/code/Amp1394Time.cpp.o.requires

.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/requires

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/clean:
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib && $(CMAKE_COMMAND) -P CMakeFiles/Amp1394.dir/cmake_clean.cmake
.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/clean

components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/depend:
	cd /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394 /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/components/code/Amp1394/lib /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib /home/cos/pmd-dvrk-nodocker/src/cisst-saw/sawRobotIO1394/cmake-build-debug/components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : components/code/Amp1394/lib/CMakeFiles/Amp1394.dir/depend

