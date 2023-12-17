# Copyright (c) 2020 Jonathan Moallem (@J-Mo63) & Aryeh Zinn (@Raelr)
#
# This code is released under an unmodified zlib license.
# For conditions of distribution and use, please see:
#     https://opensource.org/licenses/Zlib

# Define custom functions
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
platformpth = $(subst /,$(PATHSEP),$1)

# Set global macros
buildDir := bin
executable := app
target := $(buildDir)/$(executable)
sources := $(call rwildcard,src/,*.cpp)
objects := $(patsubst src/%, $(buildDir)/%, $(patsubst %.cpp, %.o, $(sources)))
depends := $(patsubst %.o, %.d, $(objects))
compileFlags := -std=c++17 -I include -I ./include/ECS -I ./include/ECS/components -I /usr/include/eigen3/ -g -Wall
linkFlags = -L lib/$(platform) -l raylib

# Check for Windows
ifeq ($(OS), Windows_NT)
	# Set Windows macros
	platform := Windows
	CXX ?= g++
	linkFlags += -Wl,--allow-multiple-definition -pthread -lopengl32 -lgdi32 -lwinmm -mwindows -static -static-libgcc -static-libstdc++
	libGenDir := src
	THEN := &&
	PATHSEP := \$(BLANK)
	MKDIR := -mkdir -p
	RM := -del /q
	COPY = -robocopy "$(call platformpth,$1)" "$(call platformpth,$2)" $3
else
	# Check for MacOS/Linux
	UNAMEOS := $(shell uname)
	ifeq ($(UNAMEOS), Linux)
		# Set Linux macros
		platform := Linux
		CXX ?= g++
		linkFlags += -l GL -l m -l pthread -l dl -l rt -l X11
	endif
	ifeq ($(UNAMEOS), Darwin)
		# Set macOS macros
		platform := macOS
		CXX ?= clang++
		linkFlags += -framework CoreVideo -framework IOKit -framework Cocoa -framework GLUT -framework OpenGL
		libGenDir := src
	endif

	# Set UNIX macros
	THEN := ;
	PATHSEP := /
	MKDIR := mkdir -p
	RM := rm -rf
	COPY = cp $1$(PATHSEP)$3 $2
endif

# Lists phony targets for Makefile
.PHONY: all setup submodules execute clean

# Default target, compiles, executes and cleans
all: $(target) execute #clean

# Sets up the project for compiling, generates includes and libs
setup: include lib

# Pull and update the the build submodules
submodules:
	git submodule update --init --recursive

# Copy the relevant header files into includes
include: submodules
	$(MKDIR) $(call platformpth, ./include)
	$(call COPY,vendor/raylib/src,./include,raylib.h)
	$(call COPY,vendor/raylib/src,./include,raymath.h)
	$(call COPY,vendor/raylib-cpp/include,./include,*.hpp)

# Build the raylib static library file and copy it into lib
lib: submodules
	cd vendor/raylib/src $(THEN) "$(MAKE)" PLATFORM=PLATFORM_DESKTOP
	$(MKDIR) $(call platformpth, lib/$(platform))
	$(call COPY,vendor/raylib/$(libGenDir),lib/$(platform),libraylib.a)

# Add all rules from dependency files
-include $(depends)

# Compile objects to the build directory
$(buildDir)/%.o: src/%.cpp Makefile
	$(MKDIR) $(call platformpth, $(@D))
	$(info This works) 
	$(CXX) -MMD -MP -c $(compileFlags) $< -o $@ $(CXXFLAGS)

# Run the executable
execute:
	$(target) $(ARGS)

# Clean up all relevant files
clean:
	$(RM) $(call platformpth, $(buildDir)/*)


# Link the program and create the executable
$(target): $(objects)
	$(CXX) $(objects) -o $(target) $(linkFlags)

# Link the program and create the executable
boids: $(objects) boid_main.o
	$(CXX) $(objects) bin/boid_main.o -o $(target) $(linkFlags)

boid_main.o:
	$(CXX) -c $(compileFlags) Examples/Boids/main.cpp -o bin/boid_main.o 

# Link the program and create the executable
collision: $(objects) collision_main.o
	$(CXX) $(objects) bin/collision_main.o -o $(target) $(linkFlags)

collision_main.o:
	$(CXX) -c $(compileFlags) Examples/Collision/main.cpp -o bin/collision_main.o

# Link the program and create the executable
flow_field: $(objects) flow_field_main.o
	$(CXX) $(objects) bin/flow_field_main.o -o $(target) $(linkFlags)

flow_field_main.o:
	$(CXX) -c $(compileFlags) Examples/FlowField/main.cpp -o bin/flow_field_main.o
