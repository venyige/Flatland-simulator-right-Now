cmake_minimum_required(VERSION 2.8)

project(Flatland-simulator-right-Now)

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
  src/guard_agent.cpp
  src/keyboard.cpp
  src/driver.cpp
  )

set(HEADERS
    src/include/gui_curses.h
    src/include/guard_agent.h
    src/include/keyboard.h
    src/include/driver.h
    src/include/interface.h
  )
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC src/include)
target_link_libraries (${PROJECT_NAME} ${CURSES_LIBRARIES}  ${X11_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "fsrn" CXX_STANDARD 17 CXX_EXTENSIONS ON)
