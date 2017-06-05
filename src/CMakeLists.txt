include(FindPkgConfig)

# openhrp3
pkg_check_modules(OPENHRP3 REQUIRED openhrp3.1)
include_directories(${OPENHRP3_INCLUDE_DIRS})
link_directories(${OPENHRP3_LIBRARY_DIRS})
link_libraries(${OPENHRP3_LIBRARIES})

add_executable(testModelLoader testModelLoader.cpp)
