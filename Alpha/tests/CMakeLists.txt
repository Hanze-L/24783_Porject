# Find Google Test package
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

set(TEST_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/STL
    ${CMAKE_SOURCE_DIR}/Slice
    ${CMAKE_SOURCE_DIR}/Pathplanner
    ${GTEST_INCLUDE_DIRS}
)

set(TEST_LINK_LIBS
    GTest::GTest
    GTest::Main
    stlfileloader
    uniformslicingalg
    pathplanner
    fssimplewindow
)

# STLFileLoader tests
add_executable(test_stlfileloader test_stlfileloader.cpp)
target_include_directories(test_stlfileloader PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_stlfileloader PRIVATE ${TEST_LINK_LIBS})
add_test(NAME STLFileLoaderTest COMMAND test_stlfileloader)

# UniformSlicingAlgorithm tests
add_executable(test_uniformslicingalg test_uniformslicingalg.cpp)
target_include_directories(test_uniformslicingalg PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_uniformslicingalg PRIVATE ${TEST_LINK_LIBS})
add_test(NAME UniformSlicingAlgTest COMMAND test_uniformslicingalg)

# PathPlanner tests
add_executable(test_pathplanner test_pathplanner.cpp)
target_include_directories(test_pathplanner PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_pathplanner PRIVATE ${TEST_LINK_LIBS})
add_test(NAME PathPlannerTest COMMAND test_pathplanner)

# Add custom target to run all tests
add_custom_target(check 
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose
    DEPENDS test_stlfileloader test_uniformslicingalg test_pathplanner
)