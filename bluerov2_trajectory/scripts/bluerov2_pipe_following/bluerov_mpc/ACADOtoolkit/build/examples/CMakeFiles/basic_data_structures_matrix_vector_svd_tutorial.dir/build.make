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
include examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/flags.make

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/flags.make
examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o: ../examples/basic_data_structures/matrix_vector/svd_tutorial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o -c /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/matrix_vector/svd_tutorial.cpp

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.i"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/matrix_vector/svd_tutorial.cpp > CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.i

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.s"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hakim/Desktop/ACADOtoolkit/examples/basic_data_structures/matrix_vector/svd_tutorial.cpp -o CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.s

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.requires:

.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.requires

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.provides: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/build.make examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.provides.build
.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.provides

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.provides.build: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o


# Object files for target basic_data_structures_matrix_vector_svd_tutorial
basic_data_structures_matrix_vector_svd_tutorial_OBJECTS = \
"CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o"

# External object files for target basic_data_structures_matrix_vector_svd_tutorial
basic_data_structures_matrix_vector_svd_tutorial_EXTERNAL_OBJECTS =

../examples/basic_data_structures/matrix_vector/svd_tutorial: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o
../examples/basic_data_structures/matrix_vector/svd_tutorial: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/build.make
../examples/basic_data_structures/matrix_vector/svd_tutorial: lib/libacado_toolkit_s.so.1.2.2beta
../examples/basic_data_structures/matrix_vector/svd_tutorial: lib/libacado_casadi.a
../examples/basic_data_structures/matrix_vector/svd_tutorial: lib/libacado_qpoases.a
../examples/basic_data_structures/matrix_vector/svd_tutorial: lib/libacado_csparse.a
../examples/basic_data_structures/matrix_vector/svd_tutorial: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hakim/Desktop/ACADOtoolkit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../examples/basic_data_structures/matrix_vector/svd_tutorial"
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/build: ../examples/basic_data_structures/matrix_vector/svd_tutorial

.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/build

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/requires: examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/basic_data_structures/matrix_vector/svd_tutorial.cpp.o.requires

.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/requires

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/clean:
	cd /home/hakim/Desktop/ACADOtoolkit/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/clean

examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/depend:
	cd /home/hakim/Desktop/ACADOtoolkit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hakim/Desktop/ACADOtoolkit /home/hakim/Desktop/ACADOtoolkit/examples /home/hakim/Desktop/ACADOtoolkit/build /home/hakim/Desktop/ACADOtoolkit/build/examples /home/hakim/Desktop/ACADOtoolkit/build/examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/basic_data_structures_matrix_vector_svd_tutorial.dir/depend

