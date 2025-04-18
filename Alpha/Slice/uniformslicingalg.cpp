#include "uniformslicingalg.h"
#include <algorithm>

UniformSlicingAlgorithm::UniformSlicingAlgorithm(const std::vector<Facet>& facets) : facets_(facets) {}

UniformSlicingAlgorithm::~UniformSlicingAlgorithm() {}

void UniformSlicingAlgorithm::setToolLength(float toolLength) {
    toolLength_ = toolLength;
}

std::vector<float> UniformSlicingAlgorithm::generateSlices() {
    // Find min and max Z values (for vertical slicing along Z-axis)
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::min();

    for (const auto& facet : facets_) {
        for (int i = 0; i < 3; ++i) {
            minZ = std::min(minZ, facet.vertices[i][2]); // Use Z-coordinate (index 2)
            maxZ = std::max(maxZ, facet.vertices[i][2]); // Use Z-coordinate (index 2)
        }
    }

    // Calculate number of slices (front-to-back)
    float sliceThickness = toolLength_ * 0.75f;
    int numSlices = static_cast<int>((maxZ - minZ) / sliceThickness) + 1;

    // Generate slice planes from front to back
    std::vector<float> slices;
    for (int i = 0; i < numSlices; ++i) {
        float z = minZ + i * sliceThickness; // Generate slices from min to max Z
        slices.push_back(z);
    }

    return slices;
}

std::vector<ContourPoint> UniformSlicingAlgorithm::generateContour(float z) {
    std::vector<ContourPoint> contourPoints;

    for (const auto& facet : facets_) {
        // Check if facet intersects with the slice plane at z
        for (int i = 0; i < 3; ++i) {
            int j = (i + 1) % 3; // Next vertex index
            if ((facet.vertices[i][2] - z) * (facet.vertices[j][2] - z) < 0) {
                // Intersection occurs; calculate intersection point
                float t = (z - facet.vertices[i][2]) / (facet.vertices[j][2] - facet.vertices[i][2]);
                float x = facet.vertices[i][0] + t * (facet.vertices[j][0] - facet.vertices[i][0]);
                float y = facet.vertices[i][1] + t * (facet.vertices[j][1] - facet.vertices[i][1]);

                // Calculate normal at intersection point
                // For simplicity, use the facet's normal
                ContourPoint point;
                point.point[0] = x;
                point.point[1] = y;
                point.point[2] = z;
                for (int k = 0; k < 3; ++k) {
                    point.normal[k] = facet.normal[k];
                }

                contourPoints.push_back(point);
            }
        }
    }

    return contourPoints;
}