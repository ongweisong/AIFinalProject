cmake_minimum_required(VERSION 3.20)

project(PlatformerPathGraphDemo LANGUAGES CXX)

add_executable(PlatformerPathGraphDemo WIN32 main.cpp)
target_compile_features(PlatformerPathGraphDemo PRIVATE cxx_std_17)
target_link_libraries(PlatformerPathGraphDemo PRIVATE user32 gdi32)

if(MSVC)
    target_compile_options(PlatformerPathGraphDemo PRIVATE /W4 /permissive-)
endif()
