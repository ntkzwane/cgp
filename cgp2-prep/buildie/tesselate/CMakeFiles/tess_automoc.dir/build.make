# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/chuck/Downloads/Downloads/cgp2-prep

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chuck/Downloads/Downloads/cgp2-prep/buildie

# Utility rule file for tess_automoc.

# Include the progress variables for this target.
include tesselate/CMakeFiles/tess_automoc.dir/progress.make

tesselate/CMakeFiles/tess_automoc:
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chuck/Downloads/Downloads/cgp2-prep/buildie/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Automoc for target tess"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/buildie/tesselate && /usr/bin/cmake -E cmake_automoc /home/chuck/Downloads/Downloads/cgp2-prep/buildie/tesselate/CMakeFiles/tess_automoc.dir/ ""

tess_automoc: tesselate/CMakeFiles/tess_automoc
tess_automoc: tesselate/CMakeFiles/tess_automoc.dir/build.make
.PHONY : tess_automoc

# Rule to build all files generated by this target.
tesselate/CMakeFiles/tess_automoc.dir/build: tess_automoc
.PHONY : tesselate/CMakeFiles/tess_automoc.dir/build

tesselate/CMakeFiles/tess_automoc.dir/clean:
	cd /home/chuck/Downloads/Downloads/cgp2-prep/buildie/tesselate && $(CMAKE_COMMAND) -P CMakeFiles/tess_automoc.dir/cmake_clean.cmake
.PHONY : tesselate/CMakeFiles/tess_automoc.dir/clean

tesselate/CMakeFiles/tess_automoc.dir/depend:
	cd /home/chuck/Downloads/Downloads/cgp2-prep/buildie && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chuck/Downloads/Downloads/cgp2-prep /home/chuck/Downloads/Downloads/cgp2-prep/tesselate /home/chuck/Downloads/Downloads/cgp2-prep/buildie /home/chuck/Downloads/Downloads/cgp2-prep/buildie/tesselate /home/chuck/Downloads/Downloads/cgp2-prep/buildie/tesselate/CMakeFiles/tess_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tesselate/CMakeFiles/tess_automoc.dir/depend

