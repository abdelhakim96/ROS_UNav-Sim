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
include examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/flags.make

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/flags.make
examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o: ../examples/basic_data_structures/variables_grid/getting_started.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/variables_grid/getting_started.cpp

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/variables_grid/getting_started.cpp > CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.i

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/variables_grid/getting_started.cpp -o CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.s

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.requires:

.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.requires

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.provides: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/build.make examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.provides.build
.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.provides

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.provides.build: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o


# Object files for target basic_data_structures_variables_grid_getting_started
basic_data_structures_variables_grid_getting_started_OBJECTS = \
"CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o"

# External object files for target basic_data_structures_variables_grid_getting_started
basic_data_structures_variables_grid_getting_started_EXTERNAL_OBJECTS =

../examples/basic_data_structures/variables_grid/getting_started: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o
../examples/basic_data_structures/variables_grid/getting_started: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/build.make
../examples/basic_data_structures/variables_grid/getting_started: lib/libacado_toolkit_s.so.1.2.2beta
../examples/basic_data_structures/variables_grid/getting_started: lib/libacado_casadi.a
../examples/basic_data_structures/variables_grid/getting_started: lib/libacado_qpoases.a
../examples/basic_data_structures/variables_grid/getting_started: lib/libacado_csparse.a
../examples/basic_data_structures/variables_grid/getting_started: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/basic_data_structures/variables_grid/getting_started"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/build: ../examples/basic_data_structures/variables_grid/getting_started

.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/build

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/requires: examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/basic_data_structures/variables_grid/getting_started.cpp.o.requires

.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/requires

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/clean

examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/basic_data_structures_variables_grid_getting_started.dir/depend

