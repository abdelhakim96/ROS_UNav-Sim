# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hakim/Desktop/ACADOtoolkit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hakim/Desktop/ACADOtoolkit/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/getting_started_simple_mpc.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/getting_started_simple_mpc.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/getting_started_simple_mpc.dir/flags.make

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o: examples/CMakeFiles/getting_started_simple_mpc.dir/flags.make
examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o: ../examples/getting_started/simple_mpc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/getting_started/simple_mpc.cpp

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/getting_started/simple_mpc.cpp > CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.i

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/getting_started/simple_mpc.cpp -o CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.s

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.requires:

.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.requires

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.provides: examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/getting_started_simple_mpc.dir/build.make examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.provides.build
.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.provides

examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.provides.build: examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o


# Object files for target getting_started_simple_mpc
getting_started_simple_mpc_OBJECTS = \
"CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o"

# External object files for target getting_started_simple_mpc
getting_started_simple_mpc_EXTERNAL_OBJECTS =

../examples/getting_started/simple_mpc: examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o
../examples/getting_started/simple_mpc: examples/CMakeFiles/getting_started_simple_mpc.dir/build.make
../examples/getting_started/simple_mpc: lib/libacado_toolkit_s.so.1.2.2beta
../examples/getting_started/simple_mpc: lib/libacado_casadi.a
../examples/getting_started/simple_mpc: lib/libacado_qpoases.a
../examples/getting_started/simple_mpc: lib/libacado_csparse.a
../examples/getting_started/simple_mpc: examples/CMakeFiles/getting_started_simple_mpc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/getting_started/simple_mpc"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/getting_started_simple_mpc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/getting_started_simple_mpc.dir/build: ../examples/getting_started/simple_mpc

.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/build

examples/CMakeFiles/getting_started_simple_mpc.dir/requires: examples/CMakeFiles/getting_started_simple_mpc.dir/getting_started/simple_mpc.cpp.o.requires

.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/requires

examples/CMakeFiles/getting_started_simple_mpc.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/getting_started_simple_mpc.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/clean

examples/CMakeFiles/getting_started_simple_mpc.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/getting_started_simple_mpc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/getting_started_simple_mpc.dir/depend

