cmake_minimum_required(VERSION 2.8)

project(Flatland-simulator-right-Now)

option(BUILD_DOC "Build documentation" ON)
# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)


#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set (CURSES_NEED_NCURSES TRUE)
find_package (Curses REQUIRED)
include_directories (${CURSES_INCLUDE_DIRS})

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})
link_directories(${X11_LIBRARIES})

set(SOURCES
  src/main.cpp
  src/gui_curses.cpp
  src/keyboard.cpp
  src/guard_agent.cpp
  src/directions.cpp
  )

set(HEADERS
    src/include/gui_curses.h
    src/include/guard_agent.h
    src/include/keyboard.h
    src/include/driver.h
    src/include/interface.h
    src/include/di_math.h
    src/include/directions.h
  )
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC src/include)
target_link_libraries (${PROJECT_NAME} ${CURSES_LIBRARIES}  ${X11_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "fsrn" CXX_STANDARD 17 CXX_EXTENSIONS ON)
