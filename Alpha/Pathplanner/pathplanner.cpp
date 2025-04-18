#include "pathplanner.h"
#include <numeric>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>

// Small epsilon for floating point comparisons
constexpr float EPSILON = 1.0e-6f;

// Structure to represent a 2D point (for slice contours)
struct Point2D {
    float x, y;  // Now representing X and Y coordinates in vertical slice plane
    
    bool operator==(const Point2D& other) const {
        return std::abs(x - other.x) < EPSILON && std::abs(y - other.y) < EPSILON;
    }
    
    bool operator<(const Point2D& other) const {
        if (std::abs(x - other.x) < EPSILON)
            return y < other.y;
        return x < other.x;
    }
};

// Structure to represent an edge in 2D
struct Edge2D {
    Point2D p1, p2;
    
    bool operator==(const Edge2D& other) const {
        return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
    }
};

PathPlanner::PathPlanner(const std::vector<Facet>& facets) : facets_(facets) {}

// Function to calculate intersection of a line segment with the vertical slice plane
bool calculateIntersection(const float v1[3], const float v2[3], float sliceZ, float result[3]) {
    // Check if the line crosses the plane
    if ((v1[2] - sliceZ) * (v2[2] - sliceZ) >= 0) {
        return false;  // No intersection
    }
    
    // Calculate intersection parameter t where v1 + t*(v2-v1) intersects the plane
    float t = (sliceZ - v1[2]) / (v2[2] - v1[2]);
    
    // Calculate intersection point
    result[0] = v1[0] + t * (v2[0] - v1[0]);  // X coordinate
    result[1] = v1[1] + t * (v2[1] - v1[1]);  // Y coordinate
    result[2] = sliceZ;  // This is exactly at the slice Z
    
    return true;
}

// Function to find the centroid of a set of 2D points
Point2D findCentroid(const std::vector<Point2D>& points) {
    Point2D centroid = {0.0f, 0.0f};
    
    for (const auto& p : points) {
        centroid.x += p.x;
        centroid.y += p.y;
    }
    
    if (!points.empty()) {
        centroid.x /= points.size();
        centroid.y /= points.size();
    }
    
    return centroid;
}

// Function to calculate angle between two points relative to a center point
float calculateAngle(const Point2D& center, const Point2D& point) {
    return std::atan2(point.y - center.y, point.x - center.x);
}

// Sort points in clockwise or counter-clockwise order around their centroid
std::vector<Point2D> sortPointsRadially(const std::vector<Point2D>& points) {
    if (points.size() <= 2) {
        return points;  // No need to sort with 0, 1, or 2 points
    }
    
    // Find the centroid
    Point2D centroid = findCentroid(points);
    
    // Create a copy of points for sorting
    std::vector<Point2D> sortedPoints = points;
    
    // Sort points by angle around the centroid
    std::sort(sortedPoints.begin(), sortedPoints.end(), 
        [&centroid](const Point2D& a, const Point2D& b) {
            return calculateAngle(centroid, a) < calculateAngle(centroid, b);
        });
    
    return sortedPoints;
}

std::vector<PathPoint> PathPlanner::calculatePath(const std::vector<float>& slices) {
    std::vector<PathPoint> path;

    for (float z : slices) {  // Iterate over Z-axis slices
        std::vector<Point2D> intersectionPoints;
        std::vector<float> normalX, normalY, normalZ;  // Store normals for later averaging
        
        // Find all intersection points for this slice
        for (const auto& facet : facets_) {
            std::vector<float> intersections[3]; // Can have up to 2 intersection points per facet
            int numIntersections = 0;
            
            // Check each edge of the triangle for intersection
            for (int i = 0; i < 3; ++i) {
                int j = (i + 1) % 3;
                float intersection[3];
                
                if (calculateIntersection(facet.vertices[i], facet.vertices[j], z, intersection)) {
                    // Store the intersection point
                    intersections[numIntersections].resize(3);
                    for (int k = 0; k < 3; ++k) {
                        intersections[numIntersections][k] = intersection[k];
                    }
                    numIntersections++;
                    
                    // Also store the facet normal
                    normalX.push_back(facet.normal[0]);
                    normalY.push_back(facet.normal[1]);
                    normalZ.push_back(facet.normal[2]);
                }
            }
            
            // Add unique intersection points to our collection
            for (int i = 0; i < numIntersections; ++i) {
                Point2D p = {intersections[i][0], intersections[i][1]}; // X and Y coordinates
                
                // Check if this point is already in our collection (using approximate equality)
                bool exists = false;
                for (const auto& existing : intersectionPoints) {
                    if (std::abs(existing.x - p.x) < EPSILON && std::abs(existing.y - p.y) < EPSILON) {
                        exists = true;
                        break;
                    }
                }
                
                if (!exists) {
                    intersectionPoints.push_back(p);
                }
            }
        }
        
        // Sort points to form a contour
        std::vector<Point2D> contour = sortPointsRadially(intersectionPoints);
        
        // If we have points, create a path point for this slice
        if (!contour.empty() && !normalX.empty()) {
            // Average normals of all facets that intersect this slice
            float avgNX = std::accumulate(normalX.begin(), normalX.end(), 0.0f) / normalX.size();
            float avgNY = std::accumulate(normalY.begin(), normalY.end(), 0.0f) / normalY.size();
            float avgNZ = std::accumulate(normalZ.begin(), normalZ.end(), 0.0f) / normalZ.size();
            
            // Normalize the average normal
            float normalLength = std::sqrt(avgNX*avgNX + avgNY*avgNY + avgNZ*avgNZ);
            if (normalLength > EPSILON) {
                avgNX /= normalLength;
                avgNY /= normalLength;
                avgNZ /= normalLength;
            }

            // For each point in the contour, create a path point
            for (size_t i = 0; i < contour.size(); ++i) {
                PathPoint point;
                point.x = contour[i].x;
                point.y = contour[i].y;
                point.z = z;
                point.nx = avgNX;
                point.ny = avgNY;
                point.nz = avgNZ;
                
                path.push_back(point);
            }
            
            // Add a point to close the loop (return to the first point)
            if (!contour.empty()) {
                PathPoint closingPoint;
                closingPoint.x = contour[0].x;
                closingPoint.y = contour[0].y;
                closingPoint.z = z;
                closingPoint.nx = avgNX;
                closingPoint.ny = avgNY;
                closingPoint.nz = avgNZ;
                
                path.push_back(closingPoint);
            }
        }
    }

    return path;
}