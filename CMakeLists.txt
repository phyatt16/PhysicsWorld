cmake_minimum_required(VERSION 3.4)

project(PhysicsWorl)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 1)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

IF (NOT TARGET gtest)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(libs/googletest)
ENDIF()

add_library(${PROJECT_NAME} STATIC
    physicsworld.cpp
    physicsworld.h
    physicsvector.cpp
    physicsvector.h
    physicssphere.cpp
    physicssphere.h
    )

add_executable(PhysicsVector_UnitTest
    physicsvector_unittests.cpp
    physicsvector.cpp
    physicsvector.h
    )

target_link_libraries(PhysicsVector_UnitTest gtest gtest_main)

add_executable(PhysicsSphere_UnitTest
    physicssphere_unittests.cpp
    physicsvector.cpp
    physicsvector.h
    physicssphere.cpp
    physicssphere.h
    )

target_link_libraries(PhysicsSphere_UnitTest gtest gtest_main)

add_executable(PhysicsWorld_UnitTest
    physicsworld_unittests.cpp
    physicsworld.cpp
    physicsworld.h
    physicsvector.cpp
    physicsvector.h
    physicssphere.cpp
    physicssphere.h
    )

target_link_libraries(PhysicsWorld_UnitTest gtest gtest_main)

