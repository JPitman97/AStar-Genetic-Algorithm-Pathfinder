cmake_minimum_required(VERSION 2.6)

project(ASTARGA)

#Statically Link SFML
set(SFML_STATIC_LIBRARIES TRUE)

#Required Includes and Libraries
set(SFML_DIR "C:/SDK/SFML-2.5.1/VisualCPP/lib/cmake/SFML")

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

#Engine src
add_library(AStarGA
	src/PathFinder.cpp
	src/PathFinder.h
)

target_link_libraries(AStarGA sfml-system sfml-window sfml-graphics)

#App src
add_executable(App
	src/main.cpp
)

#Link App with engine library
target_link_libraries(App AStarGA)