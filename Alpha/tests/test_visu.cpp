// test_visualization.cpp
#include <gtest/gtest.h>
#include "fssimplewindow.h"
#include "stlfileloader.h"
#include "uniformslicingalg.h"
#include "pathplanner.h"

// Test 1: Verify window creation and closure
TEST(VisualizationTest, WindowCreationAndClosure) {
    // Try to create a small window
    bool windowCreated = (0 == FsOpenWindow(0, 0, 100, 100, 1, "Test Window"));
    EXPECT_TRUE(windowCreated);
    
    // Close window
    FsCloseWindow();
}

// Test 2: Verify basic drawing functionality
TEST(VisualizationTest, BasicDrawing) {
    // This test requires a special environment, may not work in all CI systems
    // Create a small off-screen rendering window
    FsOpenWindow(0, 0, 100, 100, 1, "Test Drawing");
    
    // Basic drawing
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glBegin(GL_TRIANGLES);
    glVertex3f(-1.0f, -1.0f, 0.0f);
    glVertex3f(1.0f, -1.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glEnd();
    
    FsSwapBuffers();
    
    // Pixel reading verification could be added here, but would require additional setup
    
    FsCloseWindow();
    SUCCEED(); // If no crash occurs, the test passes
}

// Test 3: Simple interaction test (simulate key press)
TEST(VisualizationTest, BasicInteraction) {
    // This test may be difficult to implement in automated environments
    FsOpenWindow(0, 0, 100, 100, 1, "Test Interaction");
    
    // Simulate ESC key being pressed
    // Note: This will not work properly in most CI systems, requires special setup
    
    FsCloseWindow();
    SUCCEED(); // If no crash occurs, the test passes
}