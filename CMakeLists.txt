cmake_minimum_required(VERSION 3.6)

project(SoftwareRasteriserSolution LANGUAGES CXX)

set(SDL_DIR ${CMAKE_SOURCE_DIR}/SDL2-2.0.12/)
set(SDL_LIB_DIR ${SDL_DIR}/lib/x64/)
set(SDL_DLL_DIR ${SDL_LIB_DIR})

set(SDL_LIBRARIES 
    ${SDL_LIB_DIR}/SDL2.lib
    ${SDL_LIB_DIR}/SDL2main.lib
    ${SDL_LIB_DIR}/SDL2test.lib
)

include_directories(${SDL_DIR}/include)

add_executable(SoftwareRasteriser
    rasteriser.cpp
    geometry.h
    model.cpp
    model.h
)
set_target_properties(SoftwareRasteriser PROPERTIES VS_DEBUGGER_ENVIRONMENT "PATH=${SDL_DLL_DIR};%PATH%")

target_link_libraries(SoftwareRasteriser
    ${SDL_LIBRARIES}
)
