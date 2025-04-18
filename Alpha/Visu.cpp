#define _USE_MATH_DEFINES  // Before any includes
#include <cmath>
#include "stlfileloader.h"
#include "uniformslicingalg.h"
#include "pathplanner.h"
#include "fssimplewindow.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <sstream>

// OpenGL
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define GL_SILENCE_DEPRECATION

//float M_PI = 3.14; 

// Calculate model bounds and center point
void calculateModelBounds(const std::vector<Facet>& facets, float& size, float center[3]) {
    if (facets.empty()) {
        size = 1.0f;
        center[0] = center[1] = center[2] = 0.0f;
        return;
    }

    // Initialize bounds
    float minX = facets[0].vertices[0][0];
    float maxX = minX;
    float minY = facets[0].vertices[0][1];
    float maxY = minY;
    float minZ = facets[0].vertices[0][2];
    float maxZ = minZ;

    // Find min and max values for each dimension
    for (const auto& facet : facets) {
        for (int i = 0; i < 3; ++i) {
            minX = std::min(minX, facet.vertices[i][0]);
            maxX = std::max(maxX, facet.vertices[i][0]);
            minY = std::min(minY, facet.vertices[i][1]);  // Track Y-axis
            maxY = std::max(maxY, facet.vertices[i][1]);  // Track Y-axis
            minZ = std::min(minZ, facet.vertices[i][2]);
            maxZ = std::max(maxZ, facet.vertices[i][2]);
        }
    }

    // Calculate center point
    center[0] = (minX + maxX) / 2.0f;
    center[1] = (minY + maxY) / 2.0f;  // Center along Y-axis
    center[2] = (minZ + maxZ) / 2.0f;

    // Calculate model size
    float dx = maxX - minX;
    float dy = maxY - minY;  // Size along Y-axis
    float dz = maxZ - minZ;
    size = std::max({ dx, dy, dz });

    if (size < 0.001f) size = 1.0f;  // Prevent too small models
}


// Draw axis for orientation
void drawAxis(float size) {
    glLineWidth(2.0f);

    // X-axis (red)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size, 0.0f, 0.0f);
    glEnd();

    // Y-axis (green)
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, size, 0.0f);
    glEnd();

    // Z-axis (blue)
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, size);
    glEnd();

    glLineWidth(1.0f);
}

// Draw end effector representation
void drawEndEffector(const PathPoint& point, float size) {
    glPushMatrix();

    glTranslatef(point.x, point.y, point.z);

    // Draw a sphere for the end effector
    const int slices = 12;
    const int stacks = 12;
    const float radius = size * 0.02f;

    glColor3f(1.0f, 0.2f, 0.2f); // Red color for end effector

    // Draw a simple sphere approximation
    for (int i = 0; i < stacks; ++i) {
        float phi1 = (float)i / stacks * M_PI;
        float phi2 = (float)(i + 1) / stacks * M_PI;

        for (int j = 0; j < slices; ++j) {
            float theta1 = (float)j / slices * 2.0f * M_PI;
            float theta2 = (float)(j + 1) / slices * 2.0f * M_PI;

            glBegin(GL_QUADS);

            // Vertex 1
            float x = radius * sin(phi1) * cos(theta1);
            float y = radius * sin(phi1) * sin(theta1);
            float z = radius * cos(phi1);
            glVertex3f(x, y, z);

            // Vertex 2
            x = radius * sin(phi1) * cos(theta2);
            y = radius * sin(phi1) * sin(theta2);
            z = radius * cos(phi1);
            glVertex3f(x, y, z);

            // Vertex 3
            x = radius * sin(phi2) * cos(theta2);
            y = radius * sin(phi2) * sin(theta2);
            z = radius * cos(phi2);
            glVertex3f(x, y, z);

            // Vertex 4
            x = radius * sin(phi2) * cos(theta1);
            y = radius * sin(phi2) * sin(theta1);
            z = radius * cos(phi2);
            glVertex3f(x, y, z);

            glEnd();
        }
    }

    // Draw a line showing the normal direction
    float normalScale = size * 0.05f;
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.0f); // Yellow for normal
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(point.nx * normalScale, point.ny * normalScale, point.nz * normalScale);
    glEnd();

    glPopMatrix();
}


// Draw a slice plane with transparency (updated for vertical slicing along Z-axis)
void drawSlicePlane(float z, float modelSize, bool highlight = false) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (highlight) {
        glColor4f(0.0f, 0.8f, 0.8f, 0.5f); // Cyan highlight
    }
    else {
        glColor4f(0.5f, 0.5f, 0.7f, 0.2f); // Normal color
    }

    const float halfSize = modelSize;  // Full model coverage

    // Draw a vertical plane perpendicular to the Z-axis at position z
    glBegin(GL_QUADS);
    glVertex3f(-halfSize, -halfSize, z);
    glVertex3f(halfSize, -halfSize, z);
    glVertex3f(halfSize, halfSize, z);
    glVertex3f(-halfSize, halfSize, z);
    glEnd();

    // Border
    glColor4f(0.7f, 0.7f, 0.9f, 0.8f);
    glBegin(GL_LINE_LOOP);
    glVertex3f(-halfSize, -halfSize, z);
    glVertex3f(halfSize, -halfSize, z);
    glVertex3f(halfSize, halfSize, z);
    glVertex3f(-halfSize, halfSize, z);
    glEnd();

    glDisable(GL_BLEND);
}


// Draw path between points
void drawPath(const std::vector<PathPoint>& path, size_t currentIndex) {
    // Draw all path points
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.6f, 1.0f); // Light blue for all points
    for (const auto& point : path) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    // Draw path lines
    glLineWidth(1.5f);
    glBegin(GL_LINE_STRIP);
    glColor3f(0.0f, 0.8f, 1.0f); // Slightly brighter blue for lines
    for (const auto& point : path) {
        glVertex3f(point.x, point.y, point.z);
    }
    glEnd();

    // Draw completed path segments with different color
    if (currentIndex > 0) {
        glLineWidth(2.5f);
        glBegin(GL_LINE_STRIP);
        glColor3f(0.0f, 1.0f, 0.5f); // Green for completed path
        for (size_t i = 0; i < currentIndex && i < path.size(); ++i) {
            glVertex3f(path[i].x, path[i].y, path[i].z);
        }
        glEnd();
    }

    glLineWidth(1.0f);
}

// Simple function to display text using OpenGL lines
void drawText(int x, int y, const std::string& text) {
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int width, height;
    FsGetWindowSize(width, height);
    glOrtho(0, width, height, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    
    glColor3f(1.0f, 1.0f, 1.0f);

    
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "UI Text: " << text << std::endl;
        firstCall = false;
    }

    
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    glVertex2i(x, y);
    glEnd();

    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

// Main program
int main() {
    // Initialize window
    int windowWidth = 1024, windowHeight = 768;
    FsOpenWindow(16, 16, windowWidth, windowHeight, 1, "STL and Basic Path Visualizer");

    // Load STL file
    STLFileLoader loader;
    bool loaded = false;

    while (!loaded) {
        std::cout << "Enter path to STL file: ";
        std::string filename;
        std::cin >> filename;

        if (!isValidSTLFile(filename)) {
            std::cout << "Invalid STL file. Please try again." << std::endl;
            continue;
        }

        loader.setFilename(filename);
        loaded = loader.loadSTLFile();

        if (!loaded) {
            std::cout << "Failed to load file. Please try again." << std::endl;
        }
    }

    std::cout << "File loaded successfully with " << loader.getFacets().size() << " facets." << std::endl;

    // Get tool length
    std::cout << "Enter tool length (in model units): ";
    float toolLength;
    std::cin >> toolLength;

    // Generate slices
    UniformSlicingAlgorithm slicer(loader.getFacets());
    slicer.setToolLength(toolLength);
    std::vector<float> slices = slicer.generateSlices();

    std::cout << "Generated " << slices.size() << " slices." << std::endl;

    // Calculate end effector path
    PathPlanner planner(loader.getFacets());
    std::vector<PathPoint> path = planner.calculatePath(slices);

    std::cout << "Generated path with " << path.size() << " points." << std::endl;

    // Calculate model bounds for visualization
    float modelSize, modelCenter[3];
    calculateModelBounds(loader.getFacets(), modelSize, modelCenter);

    std::cout << "Model size: " << modelSize << std::endl;
    std::cout << "Model center: (" << modelCenter[0] << ", " << modelCenter[1] << ", " << modelCenter[2] << ")" << std::endl;

    // Setup view parameters
    float rotX = 20.0f, rotY = 30.0f;
    float zoom = modelSize * 2.0f;
    bool wireframeMode = false;
    bool showSlices = true;
    bool showPath = true;
    bool showEndEffector = true;
    bool showAxis = true;
    bool showHelp = true;
    bool animatePath = false;
    int animationSpeed = 1;
    size_t currentPathIndex = 0;
    int activeSliceIndex = -1;  // No active slice initially
    int animationTimer = 0;

    // Display controls
    std::cout << "\nControls:" << std::endl;
    std::cout << "Arrows: Rotate model" << std::endl;
    std::cout << "Z/X: Zoom in/out" << std::endl;
    std::cout << "W: Toggle wireframe mode" << std::endl;
    std::cout << "S: Toggle slice planes" << std::endl;
    std::cout << "P: Toggle path" << std::endl;
    std::cout << "E: Toggle end effector" << std::endl;
    std::cout << "A: Toggle axis" << std::endl;
    std::cout << "Space: Start/Stop animation" << std::endl;
    std::cout << "+/-: Increase/Decrease animation speed" << std::endl;
    std::cout << "H: Toggle help" << std::endl;
    std::cout << "Up/Down: Select active slice" << std::endl;
    std::cout << "ESC: Exit" << std::endl;

    // Main loop
    while (true) {
        FsPollDevice();
        int key = FsInkey();

        // Handle key inputs
        if (FSKEY_ESC == key) break;

        // Rotation
        if (FSKEY_LEFT == key) rotY -= 5.0f;
        if (FSKEY_RIGHT == key) rotY += 5.0f;
        if (FSKEY_UP == key) rotX -= 5.0f;
        if (FSKEY_DOWN == key) rotX += 5.0f;

        // Zoom
        if (FSKEY_Z == key) zoom *= 0.9f;
        if (FSKEY_X == key) zoom *= 1.1f;

        // Toggle wireframe
        if (FSKEY_W == key) wireframeMode = !wireframeMode;

        // Toggle slices
        if (FSKEY_S == key) showSlices = !showSlices;

        // Toggle path
        if (FSKEY_P == key) showPath = !showPath;

        // Toggle end effector
        if (FSKEY_E == key) showEndEffector = !showEndEffector;

        // Toggle axis
        if (FSKEY_A == key) showAxis = !showAxis;

        // Toggle help
        if (FSKEY_H == key) showHelp = !showHelp;

        // Animation control
        if (FSKEY_SPACE == key) animatePath = !animatePath;

        // Animation speed
        if (FSKEY_PLUS == key || key == '=') {  
            animationSpeed = std::min(10, animationSpeed + 1);
        }
        if (FSKEY_MINUS == key || key == '-') {  
            animationSpeed = std::max(1, animationSpeed - 1);
        }

        // Slice navigation
        if (FSKEY_PAGEUP == key) {
            if (slices.size() > 0) {
                activeSliceIndex = (activeSliceIndex + 1) % slices.size();
            }
        }
        if (FSKEY_PAGEDOWN == key) {
            if (slices.size() > 0) {
                activeSliceIndex = (activeSliceIndex - 1 + slices.size()) % slices.size();
            }
        }

        // Update animation
        if (animatePath && !path.empty()) {
            animationTimer++;
            if (animationTimer >= (11 - animationSpeed)) {
                animationTimer = 0;
                currentPathIndex = (currentPathIndex + 1) % path.size();
            }
        }

        // Clear screen with dark background
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Enable depth testing
        glEnable(GL_DEPTH_TEST);

        // Set viewport
        glViewport(0, 0, windowWidth, windowHeight);

        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect = (float)windowWidth / (float)windowHeight;

        
        float fovY = 45.0f;
        float nearPlane = 0.1f;  // Changed from 'near'
        float farPlane = zoom * 10.0f;  // Changed from 'far'


        
        float tanHalfFovy = tanf((fovY * 3.14159f / 180.0f) / 2.0f);
        float f = 1.0f / tanHalfFovy;

        
        float perspective[16] = {
            f / aspect, 0, 0, 0,
            0, f, 0, 0,
            0, 0, (farPlane + nearPlane) / (nearPlane - farPlane), -1,
            0, 0, (2 * farPlane * nearPlane) / (nearPlane - farPlane), 0
        };  
        glMultMatrixf(perspective);

        // Set modelview
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Position camera
        glTranslatef(0, 0, -zoom);

        // Apply rotation
        glRotatef(rotX, 1, 0, 0);
        glRotatef(rotY, 0, 1, 0);

        // Draw coordinate axis if enabled
        if (showAxis) {
            drawAxis(modelSize * 0.5f);
        }

        // Set model drawing mode
        if (wireframeMode) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // Enable lighting for solid mode
        if (!wireframeMode) {
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);

            // Set light position
            GLfloat lightPos[] = { zoom * 2.0f, zoom * 2.0f, zoom * 2.0f, 1.0f };
            glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

            // Set ambient light
            GLfloat ambientLight[] = { 0.3f, 0.3f, 0.3f, 1.0f };
            glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);

            // Set diffuse light
            GLfloat diffuseLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
            glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
        }

        // Center the model
        glTranslatef(-modelCenter[0], -modelCenter[1], -modelCenter[2]);

        // Draw model facets
        const std::vector<Facet>& facets = loader.getFacets();

        if (!wireframeMode) {
            // Material properties for solid mode
            GLfloat modelColor[] = { 0.8f, 0.8f, 0.8f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, modelColor);
        }
        else {
            // Color for wireframe mode
            glColor3f(0.7f, 0.7f, 0.7f);
        }

        glBegin(GL_TRIANGLES);
        for (const auto& facet : facets) {
            // Set normal for the facet
            glNormal3fv(facet.normal);

            // Draw vertices
            for (int i = 0; i < 3; ++i) {
                glVertex3fv(facet.vertices[i]);
            }
        }
        glEnd();

        // Disable lighting after drawing the model
        glDisable(GL_LIGHTING);

        // Reset polygon mode to fill for other elements
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Draw slice planes if enabled
        if (showSlices) {
            for (size_t i = 0; i < slices.size(); ++i) {
                // Highlight the active slice
                bool isActive = (int)i == activeSliceIndex;
                drawSlicePlane(slices[i], modelSize, isActive);
            }
        }

        // Draw path if enabled
        if (showPath && !path.empty()) {
            drawPath(path, currentPathIndex);
        }

        // Draw end effector if enabled
        if (showEndEffector && !path.empty() && currentPathIndex < path.size()) {
            drawEndEffector(path[currentPathIndex], modelSize);
        }

        // Display control information in console (first run only)
        static bool firstRun = true;
        if (firstRun && showHelp) {
            std::cout << "\nCurrent Control Status:" << std::endl;
            std::cout << "Rotation: Use arrow keys" << std::endl;
            std::cout << "Zoom: Z/X" << std::endl;
            std::cout << "Wireframe mode: " << (wireframeMode ? "ON" : "OFF") << " (Toggle with W key)" << std::endl;
            std::cout << "Show slices: " << (showSlices ? "ON" : "OFF") << " (Toggle with S key)" << std::endl;
            std::cout << "Show path: " << (showPath ? "ON" : "OFF") << " (Toggle with P key)" << std::endl;
            std::cout << "Show end effector: " << (showEndEffector ? "ON" : "OFF") << " (Toggle with E key)" << std::endl;
            std::cout << "Show axis: " << (showAxis ? "ON" : "OFF") << " (Toggle with A key)" << std::endl;
            std::cout << "Animation: " << (animatePath ? "ON" : "OFF") << " (Toggle with Space key)" << std::endl;
            std::cout << "Animation speed: " << animationSpeed << " (Adjust with +/- keys)" << std::endl;
            std::cout << "Current slice: " << (activeSliceIndex >= 0 ? std::to_string(activeSliceIndex) : "None") << std::endl;
            std::cout << "Press ESC to exit" << std::endl;
            firstRun = false;
        }

        // Output to console when status changes
        static bool lastShowHelp = showHelp;
        static bool lastWireframeMode = wireframeMode;
        static bool lastShowSlices = showSlices;
        static bool lastShowPath = showPath;
        static bool lastShowEndEffector = showEndEffector;
        static bool lastShowAxis = showAxis;
        static bool lastAnimatePath = animatePath;
        static int lastAnimationSpeed = animationSpeed;
        static int lastActiveSliceIndex = activeSliceIndex;

        if (lastShowHelp != showHelp ||
            lastWireframeMode != wireframeMode ||
            lastShowSlices != showSlices ||
            lastShowPath != showPath ||
            lastShowEndEffector != showEndEffector ||
            lastShowAxis != showAxis ||
            lastAnimatePath != animatePath ||
            lastAnimationSpeed != animationSpeed ||
            lastActiveSliceIndex != activeSliceIndex) {

            std::cout << "\nStatus Update:" << std::endl;
            if (lastWireframeMode != wireframeMode)
                std::cout << "Wireframe mode: " << (wireframeMode ? "ON" : "OFF") << std::endl;
            if (lastShowSlices != showSlices)
                std::cout << "Show slices: " << (showSlices ? "ON" : "OFF") << std::endl;
            if (lastShowPath != showPath)
                std::cout << "Show path: " << (showPath ? "ON" : "OFF") << std::endl;
            if (lastShowEndEffector != showEndEffector)
                std::cout << "Show end effector: " << (showEndEffector ? "ON" : "OFF") << std::endl;
            if (lastShowAxis != showAxis)
                std::cout << "Show axis: " << (showAxis ? "ON" : "OFF") << std::endl;
            if (lastAnimatePath != animatePath)
                std::cout << "Animation: " << (animatePath ? "ON" : "OFF") << std::endl;
            if (lastAnimationSpeed != animationSpeed)
                std::cout << "Animation speed: " << animationSpeed << std::endl;
            if (lastActiveSliceIndex != activeSliceIndex)
                std::cout << "Current slice: " << (activeSliceIndex >= 0 ? std::to_string(activeSliceIndex) : "None") << std::endl;

            lastShowHelp = showHelp;
            lastWireframeMode = wireframeMode;
            lastShowSlices = showSlices;
            lastShowPath = showPath;
            lastShowEndEffector = showEndEffector;
            lastShowAxis = showAxis;
            lastAnimatePath = animatePath;
            lastAnimationSpeed = animationSpeed;
            lastActiveSliceIndex = activeSliceIndex;
        }

        // Display current end effector position (only when position changes)
        static int lastPathIndex = -1;
        if (currentPathIndex < path.size() && lastPathIndex != (int)currentPathIndex) {
            const auto& point = path[currentPathIndex];
            std::cout << "End effector position: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
            lastPathIndex = currentPathIndex;
        }

        // Swap buffers and sleep
        FsSwapBuffers();
        FsSleep(16);  // ~60 FPS
    }

    FsCloseWindow();
    return 0;
}