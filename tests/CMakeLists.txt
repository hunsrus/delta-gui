cmake_minimum_required(VERSION 3.5) # FetchContent is available in 3.11+
project(new-kinematics)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB SOURCES
  OctoKinematics.h
  OctoKinematics.cpp
  new_kinematics.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
${SOURCES}
)

target_link_libraries(${PROJECT_NAME} pigpio rt pthread)