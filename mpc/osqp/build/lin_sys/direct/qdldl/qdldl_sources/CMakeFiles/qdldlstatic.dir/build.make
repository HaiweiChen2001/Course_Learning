# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/cmake-3.31.0-rc2-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.31.0-rc2-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/harvey/Downloads/mpc/osqp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/harvey/Downloads/mpc/osqp/build

# Include any dependencies generated for this target.
include lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/compiler_depend.make

# Include the progress variables for this target.
include lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/progress.make

# Include the compile flags for this target's objects.
include lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/flags.make

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/codegen:
.PHONY : lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/codegen

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o: lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/flags.make
lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o: /home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c
lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o: lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/harvey/Downloads/mpc/osqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o"
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o -MF CMakeFiles/qdldlstatic.dir/src/qdldl.c.o.d -o CMakeFiles/qdldlstatic.dir/src/qdldl.c.o -c /home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/qdldlstatic.dir/src/qdldl.c.i"
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c > CMakeFiles/qdldlstatic.dir/src/qdldl.c.i

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/qdldlstatic.dir/src/qdldl.c.s"
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c -o CMakeFiles/qdldlstatic.dir/src/qdldl.c.s

# Object files for target qdldlstatic
qdldlstatic_OBJECTS = \
"CMakeFiles/qdldlstatic.dir/src/qdldl.c.o"

# External object files for target qdldlstatic
qdldlstatic_EXTERNAL_OBJECTS =

lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a: lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/src/qdldl.c.o
lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a: lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/build.make
lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a: lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/harvey/Downloads/mpc/osqp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library out/libqdldl.a"
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -P CMakeFiles/qdldlstatic.dir/cmake_clean_target.cmake
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qdldlstatic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/build: lin_sys/direct/qdldl/qdldl_sources/out/libqdldl.a
.PHONY : lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/build

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/clean:
	cd /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources && $(CMAKE_COMMAND) -P CMakeFiles/qdldlstatic.dir/cmake_clean.cmake
.PHONY : lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/clean

lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/depend:
	cd /home/harvey/Downloads/mpc/osqp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/harvey/Downloads/mpc/osqp /home/harvey/Downloads/mpc/osqp/lin_sys/direct/qdldl/qdldl_sources /home/harvey/Downloads/mpc/osqp/build /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources /home/harvey/Downloads/mpc/osqp/build/lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : lin_sys/direct/qdldl/qdldl_sources/CMakeFiles/qdldlstatic.dir/depend

