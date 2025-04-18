add_library(uniformslicingalg uniformslicingalg.cpp uniformslicingalg.h)

target_link_libraries(uniformslicingalg stlfileloader)

target_include_directories(uniformslicingalg
    PUBLIC 
        ${CMAKE_CURRENT_SOURCE_DIR}  # For uniformslicingalg.h
        ${CMAKE_SOURCE_DIR}/STL      # For stlfileloader.h
)
