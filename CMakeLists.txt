cmake_minimum_required(VERSION 3.4)

project(PhysicsWorld)

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
    physicscylinder.cpp
    physicscylinder.h
    physicsbox.cpp
    physicsbox.h
    physicsobject.cpp
    physicsobject.h
    physicsjoint.cpp
    physicsjoint.h
    )

add_executable(PhysicsVector_UnitTest
    physicsvector_unittests.cpp
    physicsvector.cpp
    physicsvector.h
    )

target_link_libraries(PhysicsVector_UnitTest gtest gtest_main)

add_executable(PhysicsWorld_UnitTest
    physicsworld_unittests.cpp
    )

target_link_libraries(PhysicsWorld_UnitTest PhysicsWorld gtest gtest_main)

