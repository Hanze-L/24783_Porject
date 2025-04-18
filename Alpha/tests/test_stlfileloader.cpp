#include <gtest/gtest.h>
#include "stlfileloader.h"
#include <fstream>
#include <cstdio>

// Create a simple binary STL file for testing
std::string createSimpleSTLFile() {
    std::string filePath = "test_simple.stl";
    std::ofstream file(filePath, std::ios::binary);
    
    // 80-byte header
    char header[80] = "Simple STL file for testing";
    file.write(header, 80);
    
    // Triangle count (1)
    uint32_t numTriangles = 1;
    file.write(reinterpret_cast<char*>(&numTriangles), 4);
    
    // One triangle
    float normal[3] = {0.0f, 0.0f, 1.0f};  // Normal vector
    float v1[3] = {0.0f, 0.0f, 0.0f};      // Vertex 1
    float v2[3] = {1.0f, 0.0f, 0.0f};      // Vertex 2
    float v3[3] = {0.0f, 1.0f, 0.0f};      // Vertex 3
    uint16_t attribCount = 0;              // Attribute byte count
    
    file.write(reinterpret_cast<char*>(normal), 12);
    file.write(reinterpret_cast<char*>(v1), 12);
    file.write(reinterpret_cast<char*>(v2), 12);
    file.write(reinterpret_cast<char*>(v3), 12);
    file.write(reinterpret_cast<char*>(&attribCount), 2);
    
    file.close();
    return filePath;
}

// Create an invalid STL file
std::string createInvalidSTLFile() {
    std::string filePath = "test_invalid.stl";
    std::ofstream file(filePath);
    file << "This is not a valid STL file";
    file.close();
    return filePath;
}

// Test 1: Load a valid STL file
TEST(STLFileLoaderTest, LoadValidFile) {
    std::string validFile = createSimpleSTLFile();
    
    // Check if file is valid
    EXPECT_TRUE(isValidSTLFile(validFile));
    
    // Load the file
    STLFileLoader loader(validFile);
    EXPECT_TRUE(loader.loadSTLFile());
    
    // Verify facet count
    EXPECT_EQ(loader.getFacets().size(), 1);
    
    // Verify facet data
    const auto& facets = loader.getFacets();
    ASSERT_EQ(facets.size(), 1);
    
    // Verify normal vector
    EXPECT_FLOAT_EQ(facets[0].normal[0], 0.0f);
    EXPECT_FLOAT_EQ(facets[0].normal[1], 0.0f);
    EXPECT_FLOAT_EQ(facets[0].normal[2], 1.0f);
    
    // Verify vertices
    EXPECT_FLOAT_EQ(facets[0].vertices[0][0], 0.0f);
    EXPECT_FLOAT_EQ(facets[0].vertices[0][1], 0.0f);
    EXPECT_FLOAT_EQ(facets[0].vertices[0][2], 0.0f);
    
    // Clean up
    std::remove(validFile.c_str());
}

// Test 2: Load an invalid STL file
TEST(STLFileLoaderTest, LoadInvalidFile) {
    std::string invalidFile = createInvalidSTLFile();
    
    // Check if file is invalid
    EXPECT_FALSE(isValidSTLFile(invalidFile));
    
    // Try to load invalid file
    STLFileLoader loader(invalidFile);
    EXPECT_FALSE(loader.loadSTLFile());
    
    // Clean up
    std::remove(invalidFile.c_str());
}

// Test 3: Handle non-existent file
TEST(STLFileLoaderTest, HandleNonExistentFile) {
    std::string nonExistentFile = "this_file_does_not_exist.stl";
    
    // Check if file is invalid
    EXPECT_FALSE(isValidSTLFile(nonExistentFile));
    
    // Try to load non-existent file
    STLFileLoader loader(nonExistentFile);
    EXPECT_FALSE(loader.loadSTLFile());
}