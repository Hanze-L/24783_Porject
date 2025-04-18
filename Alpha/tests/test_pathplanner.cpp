#include <gtest/gtest.h>
#include "pathplanner.h"
#include <vector>
#include <cmath>

// Create a simple cube facet collection for testing
std::vector<Facet> createCubeFacets() {
    std::vector<Facet> facets;
    
    // Cube from (0,0,0) to (1,1,1)
    
    // Bottom face z=0
    Facet f1;
    f1.normal[0] = 0.0f; f1.normal[1] = 0.0f; f1.normal[2] = -1.0f;
    f1.vertices[0][0] = 0.0f; f1.vertices[0][1] = 0.0f; f1.vertices[0][2] = 0.0f;
    f1.vertices[1][0] = 1.0f; f1.vertices[1][1] = 0.0f; f1.vertices[1][2] = 0.0f;
    f1.vertices[2][0] = 1.0f; f1.vertices[2][1] = 1.0f; f1.vertices[2][2] = 0.0f;
    facets.push_back(f1);
    
    Facet f2;
    f2.normal[0] = 0.0f; f2.normal[1] = 0.0f; f2.normal[2] = -1.0f;
    f2.vertices[0][0] = 0.0f; f2.vertices[0][1] = 0.0f; f2.vertices[0][2] = 0.0f;
    f2.vertices[1][0] = 1.0f; f2.vertices[1][1] = 1.0f; f2.vertices[1][2] = 0.0f;
    f2.vertices[2][0] = 0.0f; f2.vertices[2][1] = 1.0f; f2.vertices[2][2] = 0.0f;
    facets.push_back(f2);
    
    // Top face z=1
    Facet f3;
    f3.normal[0] = 0.0f; f3.normal[1] = 0.0f; f3.normal[2] = 1.0f;
    f3.vertices[0][0] = 0.0f; f3.vertices[0][1] = 0.0f; f3.vertices[0][2] = 1.0f;
    f3.vertices[1][0] = 1.0f; f3.vertices[1][1] = 1.0f; f3.vertices[1][2] = 1.0f;
    f3.vertices[2][0] = 1.0f; f3.vertices[2][1] = 0.0f; f3.vertices[2][2] = 1.0f;
    facets.push_back(f3);
    
    Facet f4;
    f4.normal[0] = 0.0f; f4.normal[1] = 0.0f; f4.normal[2] = 1.0f;
    f4.vertices[0][0] = 0.0f; f4.vertices[0][1] = 0.0f; f4.vertices[0][2] = 1.0f;
    f4.vertices[1][0] = 0.0f; f4.vertices[1][1] = 1.0f; f4.vertices[1][2] = 1.0f;
    f4.vertices[2][0] = 1.0f; f4.vertices[2][1] = 1.0f; f4.vertices[2][2] = 1.0f;
    facets.push_back(f4);
    
    // Add four side faces (simplified, only adding front and right faces)
    Facet f5;
    f5.normal[0] = 0.0f; f5.normal[1] = -1.0f; f5.normal[2] = 0.0f;
    f5.vertices[0][0] = 0.0f; f5.vertices[0][1] = 0.0f; f5.vertices[0][2] = 0.0f;
    f5.vertices[1][0] = 1.0f; f5.vertices[1][1] = 0.0f; f5.vertices[1][2] = 0.0f;
    f5.vertices[2][0] = 1.0f; f5.vertices[2][1] = 0.0f; f5.vertices[2][2] = 1.0f;
    facets.push_back(f5);
    
    Facet f6;
    f6.normal[0] = 0.0f; f6.normal[1] = -1.0f; f6.normal[2] = 0.0f;
    f6.vertices[0][0] = 0.0f; f6.vertices[0][1] = 0.0f; f6.vertices[0][2] = 0.0f;
    f6.vertices[1][0] = 1.0f; f6.vertices[1][1] = 0.0f; f6.vertices[1][2] = 1.0f;
    f6.vertices[2][0] = 0.0f; f6.vertices[2][1] = 0.0f; f6.vertices[2][2] = 1.0f;
    facets.push_back(f6);
    
    return facets;
}

// Test 1: Verify path generation
TEST(PathPlannerTest, PathGeneration) {
    auto cubeFacets = createCubeFacets();
    
    PathPlanner planner(cubeFacets);
    
    // Generate two slices for the cube
    std::vector<float> slices = {0.25f, 0.75f};
    
    auto path = planner.calculatePath(slices);
    
    // Verify path was generated
    EXPECT_FALSE(path.empty());
    
    // Verify each slice z-value corresponds to input slices
    std::vector<float> uniqueZValues;
    for (const auto& point : path) {
        // If this z-value is not yet in uniqueZValues
        if (std::find(uniqueZValues.begin(), uniqueZValues.end(), point.z) == uniqueZValues.end()) {
            uniqueZValues.push_back(point.z);
        }
    }
    
    // Verify we found all expected z-values
    ASSERT_EQ(uniqueZValues.size(), slices.size());
    for (size_t i = 0; i < uniqueZValues.size(); ++i) {
        EXPECT_FLOAT_EQ(uniqueZValues[i], slices[i]);
    }
}

// Test 2: Verify path points form a closed contour for each slice
TEST(PathPlannerTest, ClosedContours) {
    auto cubeFacets = createCubeFacets();
    
    PathPlanner planner(cubeFacets);
    
    // Create one slice
    std::vector<float> slices = {0.5f};
    
    auto path = planner.calculatePath(slices);
    
    // Verify path is not empty
    ASSERT_FALSE(path.empty());
    
    // Find indices of path points for each z-value
    std::vector<size_t> pathIndices;
    float currentZ = path[0].z;
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i].z == currentZ) {
            pathIndices.push_back(i);
        }
    }
    
    // Verify at least 3 points form a contour
    ASSERT_GE(pathIndices.size(), 3);
    
    // Verify first and last points are the same (closed contour)
    size_t lastIndex = pathIndices.back();
    size_t firstIndex = pathIndices.front();
    
    EXPECT_NEAR(path[firstIndex].x, path[lastIndex].x, 0.001f);
    EXPECT_NEAR(path[firstIndex].y, path[lastIndex].y, 0.001f);
    EXPECT_FLOAT_EQ(path[firstIndex].z, path[lastIndex].z);
}

// Test 3: Verify path point normal vectors are unit vectors
TEST(PathPlannerTest, NormalVectors) {
    auto cubeFacets = createCubeFacets();
    
    PathPlanner planner(cubeFacets);
    
    // Create slice
    std::vector<float> slices = {0.5f};
    
    auto path = planner.calculatePath(slices);
    
    // Verify path is not empty
    ASSERT_FALSE(path.empty());
    
    // Verify each path point's normal vector is a unit vector
    for (const auto& point : path) {
        float length = std::sqrt(point.nx * point.nx + point.ny * point.ny + point.nz * point.nz);
        EXPECT_NEAR(length, 1.0f, 0.001f);
    }
}