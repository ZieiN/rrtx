project(RRTx)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -unroll-loops -Wall -Wextra")

set(SOURCES
        src/rrtx.cpp
        src/map.cpp
        src/Simplifier/simplifymax.cpp
        src/Dubins/dubins.cpp
        src/Simplifier/RandomNumbers.cpp
        src/point.cpp)
set(HEADERS
        include/constants.h
        include/rrtx.h
        include/map.h
        include/point.h
        include/Simplifier/simplifymax.h
        include/Dubins/dubins.h
        include/Simplifier/RandomNumbers.h
        )

add_executable(RRTx ${HEADERS} ${SOURCES} src/main.cpp)
