add_library(stlfileloader stlfileloader.cpp stlfileloader.h)
target_include_directories(stlfileloader
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For stlfileloader.h
)

