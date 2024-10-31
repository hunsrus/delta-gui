cmake_minimum_required(VERSION 3.5) # FetchContent is available in 3.11+
project(example)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB SOURCES
  *.h
  *.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
${SOURCES}
)

target_link_libraries(${PROJECT_NAME} pigpio rt pthread)