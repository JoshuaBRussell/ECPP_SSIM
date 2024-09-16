# Copyright (c) 2020 Jonathan Moallem (@J-Mo63) & Aryeh Zinn (@Raelr)
#
# This code is released under an unmodified zlib license.
# For conditions of distribution and use, please see:
#     https://opensource.org/licenses/Zlib



# ---- Modified from it's original version ----#

# Define custom functions
rwildcard = $(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
platformpth = $(subst /,$(PATHSEP),$1)

# Set global macros
buildDir := bin
executable := app
target := $(buildDir)/$(executable)

# ECPPS_SIM Sources
sources := $(call rwildcard,src/,*.cpp)
objects := $(patsubst src/%, $(buildDir)/%, $(patsubst %.cpp, %.o, $(sources)))
depends := $(patsubst %.o, %.d, $(objects))

include_dirs := -I include -I ./include/ECS -I ./include/ECS/components -I /usr/include/eigen3/ -I ./vendor/raylib/src -I ./vendor/raylib-cpp/include -I ./vendor/rlImGui/ -I ./vendor/imgui/ -I ./vendor/implot/
compileFlags := -std=c++17  $(include_dirs) -O1 -Wall
linkFlags = -L lib/$(platform) -l raylib -l imgui_rl_backend -l implot

# ImGui Itself
imgui_dir := ./vendor/imgui
imgui_build_dir := $(buildDir)/imgui
imgui_sources := $(imgui_dir)/imgui.cpp $(imgui_dir)/imgui_demo.cpp $(imgui_dir)/imgui_draw.cpp $(imgui_dir)/imgui_tables.cpp $(imgui_dir)/imgui_widgets.cpp 
imgui_objects := $(patsubst $(imgui_dir)/%, $(imgui_build_dir)/%, $(patsubst %.cpp, %.o, $(imgui_sources)))
depends += $(patsubst %.o, %.d, $(imgui_objects))

# ImGui Backend - Currently raylib
rlImGui_dir := ./vendor/rlImGui
rlImGui_build_dir := $(buildDir)/rlImGui
rlImGui_sources := $(rlImGui_dir)/rlImGui.cpp 
rlImGui_objects := $(patsubst $(rlImGui_dir)/%, $(rlImGui_build_dir)/%, $(patsubst %.cpp, %.o, $(rlImGui_sources)))
depends += $(patsubst %.o, %.d, $(rlImGui_objects))

# ImPlot Itself
implot_dir := ./vendor/implot
implot_build_dir := $(buildDir)/implot
implot_sources := $(implot_dir)/implot.cpp $(implot_dir)/implot_demo.cpp $(implot_dir)/implot_items.cpp 
implot_objects := $(patsubst $(implot_dir)/%, $(implot_build_dir)/%, $(patsubst %.cpp, %.o, $(implot_sources)))
depends += $(patsubst %.o, %.d, $(implot_objects))



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
		libGenDir := src
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

# Sets up the project for compiling libs
setup: libraylib.a libimgui_rl_backend.a libimplot.a

# Pull and update the the build submodules
submodules:
	git submodule update --init --recursive

# Build the raylib static library file and copy it into lib
libraylib.a:
	cd vendor/raylib/src $(THEN) "$(MAKE)" PLATFORM=PLATFORM_DESKTOP
	$(MKDIR) $(call platformpth, lib/$(platform))
	$(call COPY,vendor/raylib/$(libGenDir),lib/$(platform),libraylib.a)
	@echo ""

# Build imgui static library file and copy it into lib
libimgui_rl_backend.a: $(imgui_objects) $(rlImGui_objects)
	ar rcs lib/$(platform)/libimgui_rl_backend.a $(imgui_objects) $(rlImGui_objects)
	@echo ""

bin/imgui/%.o: $(imgui_dir)/%.cpp
	$(MKDIR) $(call platformpth, $(@D))	
	$(info $@)	
	$(CXX) -MMD -MP -c $(compileFlags) $< -o $@ $(CXXFLAGS)	
	@echo ""

bin/rlImGui/%.o: $(rlImGui_dir)/%.cpp
	$(MKDIR) $(call platformpth, $(@D))	
	$(info $@)	
	$(CXX) -MMD -MP -c $(compileFlags) $< -o $@ $(CXXFLAGS)
	@echo ""

# Build implot static library file and copy it into lib
libimplot.a: $(implot_objects)
	ar rcs lib/$(platform)/libimplot.a $(implot_objects)
	@echo ""

bin/implot/%.o: $(implot_dir)/%.cpp
	$(MKDIR) $(call platformpth, $(@D))	
	$(info $@)	
	$(CXX) -MMD -MP -c $(compileFlags) $< -o $@ $(CXXFLAGS)	
	@echo ""



# Add all rules from dependency files
-include $(depends)

# Compile objects to the build directory
$(buildDir)/%.o: src/%.cpp Makefile	
	$(MKDIR) $(call platformpth, $(@D))
	$(info $@) 
	$(CXX) -MMD -MP -c $(compileFlags) $< -o $@ $(CXXFLAGS)
	@echo ""

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

# Link the program and create the executable
pendulum: $(objects) pendulum_main.o
	$(CXX) $(objects) bin/pendulum_main.o -o $(target) $(linkFlags)

pendulum_main.o:
	$(CXX) -c $(compileFlags) Examples/Pendulum/main.cpp -o bin/pendulum_main.o

# Link the program and create the executable
double_pend: $(objects) double_pend_main.o
	$(CXX) $(objects) bin/double_pend_main.o -o $(target) $(linkFlags)

double_pend_main.o:
	$(CXX) -c $(compileFlags) Examples/DoublePendulum/main.cpp -o bin/double_pend_main.o

# Link the program and create the executable
lin_constr: $(objects) lin_constr_main.o
	$(CXX) $(objects) bin/lin_constr_main.o -o $(target) $(linkFlags)

lin_constr_main.o:
	$(CXX) -c $(compileFlags) Examples/LinearConstraint/main.cpp -o bin/lin_constr_main.o

# Link the program and create the executable
rotation: $(objects) rotation_main.o
	$(CXX) $(objects) bin/rotation_main.o -o $(target) $(linkFlags)

rotation_main.o:
	$(CXX) -c $(compileFlags) Examples/Rotation/main.cpp -o bin/rotation_main.o

# Link the program and create the executable
body_point: $(objects) body_point_main.o
	$(CXX) $(objects) bin/body_point_main.o -o $(target) $(linkFlags)

body_point_main.o:
	$(CXX) -c $(compileFlags) Examples/BodyPointConstraint/main.cpp -o bin/body_point_main.o

# Link the program and create the executable
rigid_double_pend: $(objects) libimgui_rl_backend.a libraylib.a rigid_double_pend_main.o
	$(CXX) $(objects) $(imgui_objects) bin/rigid_double_pend_main.o -o $(target) $(linkFlags)

rigid_double_pend_main.o:
	$(CXX) -c $(compileFlags) Examples/RigidBodyDoublePendulum/main.cpp -o bin/rigid_double_pend_main.o
