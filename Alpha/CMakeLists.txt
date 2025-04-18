cmake_minimum_required(VERSION 3.15)
project(Visu)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()


add_subdirectory(STL)
add_subdirectory(Slice)
add_subdirectory(Pathplanner)
add_subdirectory(../../../public/src ${CMAKE_BINARY_DIR}/public)
add_subdirectory(../../../MMLPlayer/mmlplayer ${CMAKE_BINARY_DIR}/MMLPlayer)
add_subdirectory(../../../MMLplayer/ym2612 ${CMAKE_BINARY_DIR}/ym2612)
add_subdirectory(tests) 


add_executable(Visu MACOSX_BUNDLE Visu.cpp)
target_include_directories(Visu PRIVATE
    ${CMAKE_SOURCE_DIR}/STL
    ${CMAKE_SOURCE_DIR}/Slice
    ${CMAKE_SOURCE_DIR}/Pathplanner
)

target_link_libraries(Visu PRIVATE pathplanner uniformslicingalg fssimplewindow stlfileloader)
