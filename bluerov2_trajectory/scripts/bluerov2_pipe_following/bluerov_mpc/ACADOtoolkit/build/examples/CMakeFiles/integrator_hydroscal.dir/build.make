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
include examples/CMakeFiles/integrator_hydroscal.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/integrator_hydroscal.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/integrator_hydroscal.dir/flags.make

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o: examples/CMakeFiles/integrator_hydroscal.dir/flags.make
examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o: ../examples/integrator/hydroscal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/integrator/hydroscal.cpp

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/integrator/hydroscal.cpp > CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.i

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/integrator/hydroscal.cpp -o CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.s

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.requires:

.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.requires

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.provides: examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/integrator_hydroscal.dir/build.make examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.provides.build
.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.provides

examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.provides.build: examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o


# Object files for target integrator_hydroscal
integrator_hydroscal_OBJECTS = \
"CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o"

# External object files for target integrator_hydroscal
integrator_hydroscal_EXTERNAL_OBJECTS =

../examples/integrator/hydroscal: examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o
../examples/integrator/hydroscal: examples/CMakeFiles/integrator_hydroscal.dir/build.make
../examples/integrator/hydroscal: lib/libacado_toolkit_s.so.1.2.2beta
../examples/integrator/hydroscal: lib/libacado_casadi.a
../examples/integrator/hydroscal: lib/libacado_qpoases.a
../examples/integrator/hydroscal: lib/libacado_csparse.a
../examples/integrator/hydroscal: examples/CMakeFiles/integrator_hydroscal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/integrator/hydroscal"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/integrator_hydroscal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/integrator_hydroscal.dir/build: ../examples/integrator/hydroscal

.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/build

examples/CMakeFiles/integrator_hydroscal.dir/requires: examples/CMakeFiles/integrator_hydroscal.dir/integrator/hydroscal.cpp.o.requires

.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/requires

examples/CMakeFiles/integrator_hydroscal.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/integrator_hydroscal.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/clean

examples/CMakeFiles/integrator_hydroscal.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/integrator_hydroscal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/integrator_hydroscal.dir/depend

