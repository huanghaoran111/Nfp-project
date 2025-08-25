cmake_minimum_required(VERSION 3.20)

project(NFPPRJECT LANGUAGES C CXX)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(Project_Name nfpTest)

if (MSVC)
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.29)  # VS 2019 16.11 (MSVC 19.29)
        message(FATAL_ERROR "Requires MSVC 19.29 (VS 2019 16.11+) for <format> support")
    endif()
endif()

add_executable(${Project_Name})
add_subdirectory(third_party)

target_sources(${Project_Name} PRIVATE 
    src/app.cpp 
    src/Logger.cpp
    src/data_warp.h
    src/UI.cpp 
    src/MathCore.cpp 
    src/Shape.cpp
    src/draw_warp.cpp
    src/MathCore.cpp
)
target_link_libraries(${Project_Name} PUBLIC glad glfw opengl32 imgui PRIVATE spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>)
target_include_directories(${Project_Name} PRIVATE src)
# target_include_directories()