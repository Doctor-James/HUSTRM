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
CMAKE_SOURCE_DIR = /home/zhou/RM/RM2022/AIM_RM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhou/RM/RM2022/AIM_RM/build

# Include any dependencies generated for this target.
include armor/CMakeFiles/armor.dir/depend.make

# Include the progress variables for this target.
include armor/CMakeFiles/armor.dir/progress.make

# Include the compile flags for this target's objects.
include armor/CMakeFiles/armor.dir/flags.make

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o: armor/CMakeFiles/armor.dir/flags.make
armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o: ../armor/src/ArmorScore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armor.dir/src/ArmorScore.cpp.o -c /home/zhou/RM/RM2022/AIM_RM/armor/src/ArmorScore.cpp

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor.dir/src/ArmorScore.cpp.i"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/RM/RM2022/AIM_RM/armor/src/ArmorScore.cpp > CMakeFiles/armor.dir/src/ArmorScore.cpp.i

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor.dir/src/ArmorScore.cpp.s"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/RM/RM2022/AIM_RM/armor/src/ArmorScore.cpp -o CMakeFiles/armor.dir/src/ArmorScore.cpp.s

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.requires:

.PHONY : armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.requires

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.provides: armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.requires
	$(MAKE) -f armor/CMakeFiles/armor.dir/build.make armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.provides.build
.PHONY : armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.provides

armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.provides.build: armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o


armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o: armor/CMakeFiles/armor.dir/flags.make
armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o: ../armor/src/LightBarScore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armor.dir/src/LightBarScore.cpp.o -c /home/zhou/RM/RM2022/AIM_RM/armor/src/LightBarScore.cpp

armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor.dir/src/LightBarScore.cpp.i"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/RM/RM2022/AIM_RM/armor/src/LightBarScore.cpp > CMakeFiles/armor.dir/src/LightBarScore.cpp.i

armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor.dir/src/LightBarScore.cpp.s"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/RM/RM2022/AIM_RM/armor/src/LightBarScore.cpp -o CMakeFiles/armor.dir/src/LightBarScore.cpp.s

armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.requires:

.PHONY : armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.requires

armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.provides: armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.requires
	$(MAKE) -f armor/CMakeFiles/armor.dir/build.make armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.provides.build
.PHONY : armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.provides

armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.provides.build: armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o


armor/CMakeFiles/armor.dir/src/armor.cpp.o: armor/CMakeFiles/armor.dir/flags.make
armor/CMakeFiles/armor.dir/src/armor.cpp.o: ../armor/src/armor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object armor/CMakeFiles/armor.dir/src/armor.cpp.o"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armor.dir/src/armor.cpp.o -c /home/zhou/RM/RM2022/AIM_RM/armor/src/armor.cpp

armor/CMakeFiles/armor.dir/src/armor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor.dir/src/armor.cpp.i"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/RM/RM2022/AIM_RM/armor/src/armor.cpp > CMakeFiles/armor.dir/src/armor.cpp.i

armor/CMakeFiles/armor.dir/src/armor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor.dir/src/armor.cpp.s"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/RM/RM2022/AIM_RM/armor/src/armor.cpp -o CMakeFiles/armor.dir/src/armor.cpp.s

armor/CMakeFiles/armor.dir/src/armor.cpp.o.requires:

.PHONY : armor/CMakeFiles/armor.dir/src/armor.cpp.o.requires

armor/CMakeFiles/armor.dir/src/armor.cpp.o.provides: armor/CMakeFiles/armor.dir/src/armor.cpp.o.requires
	$(MAKE) -f armor/CMakeFiles/armor.dir/build.make armor/CMakeFiles/armor.dir/src/armor.cpp.o.provides.build
.PHONY : armor/CMakeFiles/armor.dir/src/armor.cpp.o.provides

armor/CMakeFiles/armor.dir/src/armor.cpp.o.provides.build: armor/CMakeFiles/armor.dir/src/armor.cpp.o


armor/CMakeFiles/armor.dir/src/lightBar.cpp.o: armor/CMakeFiles/armor.dir/flags.make
armor/CMakeFiles/armor.dir/src/lightBar.cpp.o: ../armor/src/lightBar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object armor/CMakeFiles/armor.dir/src/lightBar.cpp.o"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armor.dir/src/lightBar.cpp.o -c /home/zhou/RM/RM2022/AIM_RM/armor/src/lightBar.cpp

armor/CMakeFiles/armor.dir/src/lightBar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor.dir/src/lightBar.cpp.i"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/RM/RM2022/AIM_RM/armor/src/lightBar.cpp > CMakeFiles/armor.dir/src/lightBar.cpp.i

armor/CMakeFiles/armor.dir/src/lightBar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor.dir/src/lightBar.cpp.s"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/RM/RM2022/AIM_RM/armor/src/lightBar.cpp -o CMakeFiles/armor.dir/src/lightBar.cpp.s

armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.requires:

.PHONY : armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.requires

armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.provides: armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.requires
	$(MAKE) -f armor/CMakeFiles/armor.dir/build.make armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.provides.build
.PHONY : armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.provides

armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.provides.build: armor/CMakeFiles/armor.dir/src/lightBar.cpp.o


armor/CMakeFiles/armor.dir/src/score.cpp.o: armor/CMakeFiles/armor.dir/flags.make
armor/CMakeFiles/armor.dir/src/score.cpp.o: ../armor/src/score.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object armor/CMakeFiles/armor.dir/src/score.cpp.o"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/armor.dir/src/score.cpp.o -c /home/zhou/RM/RM2022/AIM_RM/armor/src/score.cpp

armor/CMakeFiles/armor.dir/src/score.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/armor.dir/src/score.cpp.i"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhou/RM/RM2022/AIM_RM/armor/src/score.cpp > CMakeFiles/armor.dir/src/score.cpp.i

armor/CMakeFiles/armor.dir/src/score.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/armor.dir/src/score.cpp.s"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhou/RM/RM2022/AIM_RM/armor/src/score.cpp -o CMakeFiles/armor.dir/src/score.cpp.s

armor/CMakeFiles/armor.dir/src/score.cpp.o.requires:

.PHONY : armor/CMakeFiles/armor.dir/src/score.cpp.o.requires

armor/CMakeFiles/armor.dir/src/score.cpp.o.provides: armor/CMakeFiles/armor.dir/src/score.cpp.o.requires
	$(MAKE) -f armor/CMakeFiles/armor.dir/build.make armor/CMakeFiles/armor.dir/src/score.cpp.o.provides.build
.PHONY : armor/CMakeFiles/armor.dir/src/score.cpp.o.provides

armor/CMakeFiles/armor.dir/src/score.cpp.o.provides.build: armor/CMakeFiles/armor.dir/src/score.cpp.o


# Object files for target armor
armor_OBJECTS = \
"CMakeFiles/armor.dir/src/ArmorScore.cpp.o" \
"CMakeFiles/armor.dir/src/LightBarScore.cpp.o" \
"CMakeFiles/armor.dir/src/armor.cpp.o" \
"CMakeFiles/armor.dir/src/lightBar.cpp.o" \
"CMakeFiles/armor.dir/src/score.cpp.o"

# External object files for target armor
armor_EXTERNAL_OBJECTS =

armor/libarmor.a: armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o
armor/libarmor.a: armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o
armor/libarmor.a: armor/CMakeFiles/armor.dir/src/armor.cpp.o
armor/libarmor.a: armor/CMakeFiles/armor.dir/src/lightBar.cpp.o
armor/libarmor.a: armor/CMakeFiles/armor.dir/src/score.cpp.o
armor/libarmor.a: armor/CMakeFiles/armor.dir/build.make
armor/libarmor.a: armor/CMakeFiles/armor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhou/RM/RM2022/AIM_RM/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libarmor.a"
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && $(CMAKE_COMMAND) -P CMakeFiles/armor.dir/cmake_clean_target.cmake
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/armor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
armor/CMakeFiles/armor.dir/build: armor/libarmor.a

.PHONY : armor/CMakeFiles/armor.dir/build

armor/CMakeFiles/armor.dir/requires: armor/CMakeFiles/armor.dir/src/ArmorScore.cpp.o.requires
armor/CMakeFiles/armor.dir/requires: armor/CMakeFiles/armor.dir/src/LightBarScore.cpp.o.requires
armor/CMakeFiles/armor.dir/requires: armor/CMakeFiles/armor.dir/src/armor.cpp.o.requires
armor/CMakeFiles/armor.dir/requires: armor/CMakeFiles/armor.dir/src/lightBar.cpp.o.requires
armor/CMakeFiles/armor.dir/requires: armor/CMakeFiles/armor.dir/src/score.cpp.o.requires

.PHONY : armor/CMakeFiles/armor.dir/requires

armor/CMakeFiles/armor.dir/clean:
	cd /home/zhou/RM/RM2022/AIM_RM/build/armor && $(CMAKE_COMMAND) -P CMakeFiles/armor.dir/cmake_clean.cmake
.PHONY : armor/CMakeFiles/armor.dir/clean

armor/CMakeFiles/armor.dir/depend:
	cd /home/zhou/RM/RM2022/AIM_RM/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhou/RM/RM2022/AIM_RM /home/zhou/RM/RM2022/AIM_RM/armor /home/zhou/RM/RM2022/AIM_RM/build /home/zhou/RM/RM2022/AIM_RM/build/armor /home/zhou/RM/RM2022/AIM_RM/build/armor/CMakeFiles/armor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : armor/CMakeFiles/armor.dir/depend
