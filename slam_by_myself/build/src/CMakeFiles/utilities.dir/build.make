# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hxb/github/slam/slam_by_myself

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hxb/github/slam/slam_by_myself/build

# Include any dependencies generated for this target.
include src/CMakeFiles/utilities.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/utilities.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/utilities.dir/flags.make

src/CMakeFiles/utilities.dir/utilities.cpp.o: src/CMakeFiles/utilities.dir/flags.make
src/CMakeFiles/utilities.dir/utilities.cpp.o: ../src/utilities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hxb/github/slam/slam_by_myself/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/utilities.dir/utilities.cpp.o"
	cd /home/hxb/github/slam/slam_by_myself/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/utilities.dir/utilities.cpp.o -c /home/hxb/github/slam/slam_by_myself/src/utilities.cpp

src/CMakeFiles/utilities.dir/utilities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utilities.dir/utilities.cpp.i"
	cd /home/hxb/github/slam/slam_by_myself/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hxb/github/slam/slam_by_myself/src/utilities.cpp > CMakeFiles/utilities.dir/utilities.cpp.i

src/CMakeFiles/utilities.dir/utilities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utilities.dir/utilities.cpp.s"
	cd /home/hxb/github/slam/slam_by_myself/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hxb/github/slam/slam_by_myself/src/utilities.cpp -o CMakeFiles/utilities.dir/utilities.cpp.s

src/CMakeFiles/utilities.dir/utilities.cpp.o.requires:

.PHONY : src/CMakeFiles/utilities.dir/utilities.cpp.o.requires

src/CMakeFiles/utilities.dir/utilities.cpp.o.provides: src/CMakeFiles/utilities.dir/utilities.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/utilities.dir/build.make src/CMakeFiles/utilities.dir/utilities.cpp.o.provides.build
.PHONY : src/CMakeFiles/utilities.dir/utilities.cpp.o.provides

src/CMakeFiles/utilities.dir/utilities.cpp.o.provides.build: src/CMakeFiles/utilities.dir/utilities.cpp.o


# Object files for target utilities
utilities_OBJECTS = \
"CMakeFiles/utilities.dir/utilities.cpp.o"

# External object files for target utilities
utilities_EXTERNAL_OBJECTS =

../lib/libutilities.a: src/CMakeFiles/utilities.dir/utilities.cpp.o
../lib/libutilities.a: src/CMakeFiles/utilities.dir/build.make
../lib/libutilities.a: src/CMakeFiles/utilities.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hxb/github/slam/slam_by_myself/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../lib/libutilities.a"
	cd /home/hxb/github/slam/slam_by_myself/build/src && $(CMAKE_COMMAND) -P CMakeFiles/utilities.dir/cmake_clean_target.cmake
	cd /home/hxb/github/slam/slam_by_myself/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/utilities.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/utilities.dir/build: ../lib/libutilities.a

.PHONY : src/CMakeFiles/utilities.dir/build

src/CMakeFiles/utilities.dir/requires: src/CMakeFiles/utilities.dir/utilities.cpp.o.requires

.PHONY : src/CMakeFiles/utilities.dir/requires

src/CMakeFiles/utilities.dir/clean:
	cd /home/hxb/github/slam/slam_by_myself/build/src && $(CMAKE_COMMAND) -P CMakeFiles/utilities.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/utilities.dir/clean

src/CMakeFiles/utilities.dir/depend:
	cd /home/hxb/github/slam/slam_by_myself/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hxb/github/slam/slam_by_myself /home/hxb/github/slam/slam_by_myself/src /home/hxb/github/slam/slam_by_myself/build /home/hxb/github/slam/slam_by_myself/build/src /home/hxb/github/slam/slam_by_myself/build/src/CMakeFiles/utilities.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/utilities.dir/depend
