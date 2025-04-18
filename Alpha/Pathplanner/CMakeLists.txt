add_library(pathplanner pathplanner.cpp pathplanner.h)

target_include_directories(pathplanner 
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For pathplanner.h
        ${CMAKE_SOURCE_DIR}/Slice    # For uniformslicingalg.h
        ${CMAKE_SOURCE_DIR}/STL      # For stlfileloader.h
)
target_link_libraries(pathplanner PRIVATE uniformslicingalg stlfileloader)
