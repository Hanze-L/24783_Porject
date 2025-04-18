#include <gtest/gtest.h>
#include "uniformslicingalg.h"
#include <vector>

// Create a simple cube as test data
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
    
    // Add four side faces (simplified, only adding front face)
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

// Test 1: Verify the number of generated slices
TEST(UniformSlicingAlgTest, SliceCount) {
    auto cubeFacets = createCubeFacets();
    
    UniformSlicingAlgorithm slicer(cubeFacets);
    
    // Cube height is 1.0, test with different tool lengths
    
    // Tool length 0.5, expected slice count = (1.0 / (0.5 * 0.75)) + 1 = 3.67 -> 4
    slicer.setToolLength(0.5f);
    auto slices1 = slicer.generateSlices();
    EXPECT_EQ(slices1.size(), 4);
    
    // Tool length 1.0, expected slice count = (1.0 / (1.0 * 0.75)) + 1 = 2.33 -> 3
    slicer.setToolLength(1.0f);
    auto slices2 = slicer.generateSlices();
    EXPECT_EQ(slices2.size(), 3);
    
    // Tool length 2.0, expected slice count = (1.0 / (2.0 * 0.75)) + 1 = 1.67 -> 2
    slicer.setToolLength(2.0f);
    auto slices3 = slicer.generateSlices();
    EXPECT_EQ(slices3.size(), 2);
}

// Test 2: Verify slice position uniformity
TEST(UniformSlicingAlgTest, SliceUniformity) {
    auto cubeFacets = createCubeFacets();
    
    UniformSlicingAlgorithm slicer(cubeFacets);
    slicer.setToolLength(0.5f);
    
    auto slices = slicer.generateSlices();
    ASSERT_GE(slices.size(), 2); // At least two slices needed to test spacing
    
    // Verify slices are evenly spaced
    float expectedSpacing = 0.5f * 0.75f; // Tool length * 0.75
    for (size_t i = 1; i < slices.size(); ++i) {
        EXPECT_NEAR(slices[i] - slices[i-1], expectedSpacing, 0.001f);
    }
}

// Test 3: Test slice range
TEST(UniformSlicingAlgTest, SliceRange) {
    auto cubeFacets = createCubeFacets();
    
    UniformSlicingAlgorithm slicer(cubeFacets);
    slicer.setToolLength(0.5f);
    
    auto slices = slicer.generateSlices();
    ASSERT_FALSE(slices.empty());
    
    // Verify slice range covers the entire cube
    EXPECT_NEAR(slices.front(), 0.0f, 0.001f); // First slice should be close to min z value
    EXPECT_LE(slices.back(), 1.0f); // Last slice should not exceed max z value
}