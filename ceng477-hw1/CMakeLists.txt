cmake_minimum_required(VERSION 3.14)
project(ceng477_hw1)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(ceng477_hw1
        Camera.cpp
        Camera.h
        defs.h
        Image.cpp
        Image.h
        Light.cpp
        Light.h
        main.cpp
        Material.cpp
        Material.h
        Ray.cpp
        Ray.h
        Scene.cpp
        Scene.h
        Shape.cpp
        Shape.h
        tinyxml2.cpp
        tinyxml2.h VectorOperations.cpp VectorOperations.h)
