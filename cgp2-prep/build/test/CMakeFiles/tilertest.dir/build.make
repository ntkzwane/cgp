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
CMAKE_BINARY_DIR = /home/chuck/Downloads/Downloads/cgp2-prep/build

# Include any dependencies generated for this target.
include test/CMakeFiles/tilertest.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/tilertest.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/tilertest.dir/flags.make

test/CMakeFiles/tilertest.dir/testutil.cpp.o: test/CMakeFiles/tilertest.dir/flags.make
test/CMakeFiles/tilertest.dir/testutil.cpp.o: ../test/testutil.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chuck/Downloads/Downloads/cgp2-prep/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/tilertest.dir/testutil.cpp.o"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tilertest.dir/testutil.cpp.o -c /home/chuck/Downloads/Downloads/cgp2-prep/test/testutil.cpp

test/CMakeFiles/tilertest.dir/testutil.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tilertest.dir/testutil.cpp.i"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chuck/Downloads/Downloads/cgp2-prep/test/testutil.cpp > CMakeFiles/tilertest.dir/testutil.cpp.i

test/CMakeFiles/tilertest.dir/testutil.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tilertest.dir/testutil.cpp.s"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chuck/Downloads/Downloads/cgp2-prep/test/testutil.cpp -o CMakeFiles/tilertest.dir/testutil.cpp.s

test/CMakeFiles/tilertest.dir/testutil.cpp.o.requires:
.PHONY : test/CMakeFiles/tilertest.dir/testutil.cpp.o.requires

test/CMakeFiles/tilertest.dir/testutil.cpp.o.provides: test/CMakeFiles/tilertest.dir/testutil.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/tilertest.dir/build.make test/CMakeFiles/tilertest.dir/testutil.cpp.o.provides.build
.PHONY : test/CMakeFiles/tilertest.dir/testutil.cpp.o.provides

test/CMakeFiles/tilertest.dir/testutil.cpp.o.provides.build: test/CMakeFiles/tilertest.dir/testutil.cpp.o

test/CMakeFiles/tilertest.dir/test_mesh.cpp.o: test/CMakeFiles/tilertest.dir/flags.make
test/CMakeFiles/tilertest.dir/test_mesh.cpp.o: ../test/test_mesh.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chuck/Downloads/Downloads/cgp2-prep/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/tilertest.dir/test_mesh.cpp.o"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tilertest.dir/test_mesh.cpp.o -c /home/chuck/Downloads/Downloads/cgp2-prep/test/test_mesh.cpp

test/CMakeFiles/tilertest.dir/test_mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tilertest.dir/test_mesh.cpp.i"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chuck/Downloads/Downloads/cgp2-prep/test/test_mesh.cpp > CMakeFiles/tilertest.dir/test_mesh.cpp.i

test/CMakeFiles/tilertest.dir/test_mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tilertest.dir/test_mesh.cpp.s"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chuck/Downloads/Downloads/cgp2-prep/test/test_mesh.cpp -o CMakeFiles/tilertest.dir/test_mesh.cpp.s

test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.requires:
.PHONY : test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.requires

test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.provides: test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/tilertest.dir/build.make test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.provides.build
.PHONY : test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.provides

test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.provides.build: test/CMakeFiles/tilertest.dir/test_mesh.cpp.o

test/CMakeFiles/tilertest.dir/tilertest.cpp.o: test/CMakeFiles/tilertest.dir/flags.make
test/CMakeFiles/tilertest.dir/tilertest.cpp.o: ../test/tilertest.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chuck/Downloads/Downloads/cgp2-prep/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/tilertest.dir/tilertest.cpp.o"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tilertest.dir/tilertest.cpp.o -c /home/chuck/Downloads/Downloads/cgp2-prep/test/tilertest.cpp

test/CMakeFiles/tilertest.dir/tilertest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tilertest.dir/tilertest.cpp.i"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chuck/Downloads/Downloads/cgp2-prep/test/tilertest.cpp > CMakeFiles/tilertest.dir/tilertest.cpp.i

test/CMakeFiles/tilertest.dir/tilertest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tilertest.dir/tilertest.cpp.s"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chuck/Downloads/Downloads/cgp2-prep/test/tilertest.cpp -o CMakeFiles/tilertest.dir/tilertest.cpp.s

test/CMakeFiles/tilertest.dir/tilertest.cpp.o.requires:
.PHONY : test/CMakeFiles/tilertest.dir/tilertest.cpp.o.requires

test/CMakeFiles/tilertest.dir/tilertest.cpp.o.provides: test/CMakeFiles/tilertest.dir/tilertest.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/tilertest.dir/build.make test/CMakeFiles/tilertest.dir/tilertest.cpp.o.provides.build
.PHONY : test/CMakeFiles/tilertest.dir/tilertest.cpp.o.provides

test/CMakeFiles/tilertest.dir/tilertest.cpp.o.provides.build: test/CMakeFiles/tilertest.dir/tilertest.cpp.o

# Object files for target tilertest
tilertest_OBJECTS = \
"CMakeFiles/tilertest.dir/testutil.cpp.o" \
"CMakeFiles/tilertest.dir/test_mesh.cpp.o" \
"CMakeFiles/tilertest.dir/tilertest.cpp.o"

# External object files for target tilertest
tilertest_EXTERNAL_OBJECTS =

test/tilertest: test/CMakeFiles/tilertest.dir/testutil.cpp.o
test/tilertest: test/CMakeFiles/tilertest.dir/test_mesh.cpp.o
test/tilertest: test/CMakeFiles/tilertest.dir/tilertest.cpp.o
test/tilertest: test/CMakeFiles/tilertest.dir/build.make
test/tilertest: tesselate/libtess.a
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_system.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
test/tilertest: common/libcommon.a
test/tilertest: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.2.1
test/tilertest: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.2.1
test/tilertest: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.2.1
test/tilertest: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.2.1
test/tilertest: /usr/lib/x86_64-linux-gnu/libglut.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libXmu.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libXi.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libGLU.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libGL.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libSM.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libICE.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libX11.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libXext.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test/tilertest: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
test/tilertest: test/CMakeFiles/tilertest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tilertest"
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tilertest.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Running per-build tests"
	/home/chuck/Downloads/Downloads/cgp2-prep/build/test/tilertest --test=build

# Rule to build all files generated by this target.
test/CMakeFiles/tilertest.dir/build: test/tilertest
.PHONY : test/CMakeFiles/tilertest.dir/build

test/CMakeFiles/tilertest.dir/requires: test/CMakeFiles/tilertest.dir/testutil.cpp.o.requires
test/CMakeFiles/tilertest.dir/requires: test/CMakeFiles/tilertest.dir/test_mesh.cpp.o.requires
test/CMakeFiles/tilertest.dir/requires: test/CMakeFiles/tilertest.dir/tilertest.cpp.o.requires
.PHONY : test/CMakeFiles/tilertest.dir/requires

test/CMakeFiles/tilertest.dir/clean:
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build/test && $(CMAKE_COMMAND) -P CMakeFiles/tilertest.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/tilertest.dir/clean

test/CMakeFiles/tilertest.dir/depend:
	cd /home/chuck/Downloads/Downloads/cgp2-prep/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chuck/Downloads/Downloads/cgp2-prep /home/chuck/Downloads/Downloads/cgp2-prep/test /home/chuck/Downloads/Downloads/cgp2-prep/build /home/chuck/Downloads/Downloads/cgp2-prep/build/test /home/chuck/Downloads/Downloads/cgp2-prep/build/test/CMakeFiles/tilertest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/tilertest.dir/depend

