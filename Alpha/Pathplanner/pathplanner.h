#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include "stlfileloader.h"

// Structure to represent a point in the tool path
struct PathPoint {
    float x, y, z;        // Position
    float nx, ny, nz;     // Normal vector
};

class PathPlanner {
public:
    PathPlanner(const std::vector<Facet>& facets);
    
    std::vector<PathPoint> calculatePath(const std::vector<float>& slices);
    
private:
    std::vector<Facet> facets_;
};

#endif // PATHPLANNER_H