cmake_minimum_required(VERSION 3.5) # FetchContent is available in 3.11+
project(example)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(GRAPHICS GRAPHICS_API_OPENGL_21)

# Dependencies
find_package(raylib 4.0.0 QUIET) # QUIET or REQUIRED
if (NOT raylib_FOUND) # If there's none, fetch and build raylib
  include(FetchContent)
  FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/4.0.0.tar.gz
  )
  FetchContent_GetProperties(raylib)
  if (NOT raylib_POPULATED) # Have we downloaded raylib yet?
    set(FETCHCONTENT_QUIET NO)
    FetchContent_Populate(raylib)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
    add_subdirectory(${raylib_SOURCE_DIR} ${raylib_BINARY_DIR})
  endif()
endif()

# opencv

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Our Project

file(GLOB SOURCES
  *.h
  *.cpp
)

ADD_EXECUTABLE(${PROJECT_NAME}
${SOURCES}
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(STATUS "Detected architecture: x86_64")
    target_link_libraries(${PROJECT_NAME} raylib ${OpenCV_LIBS} pigpio)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")  # raspberry pi zero 2 w
    message(STATUS "Detected architecture: aarch46")
    target_link_libraries(${PROJECT_NAME} raylib ${OpenCV_LIBS} pigpio rt pthread)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    message(STATUS "Detected architecture: ARM")
    target_link_libraries(${PROJECT_NAME} raylib ${OpenCV_LIBS} pigpio rt pthread)
    # Aquí puedes agregar bibliotecas específicas para ARM si es necesario
    # target_link_libraries(pulse biblioteca_especifica_arm)
else()
    message(WARNING "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    target_link_libraries(${PROJECT_NAME} raylib ${OpenCV_LIBS} pigpio rt pthread)
endif()

add_custom_command(
  TARGET example POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          ${CMAKE_CURRENT_SOURCE_DIR}/resources
          ${CMAKE_CURRENT_BINARY_DIR}/resources
)