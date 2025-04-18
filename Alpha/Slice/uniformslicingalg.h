#ifndef UNIFORMSLICINGALG_H
#define UNIFORMSLICINGALG_H

#include <vector>
#include "stlfileloader.h"

struct ContourPoint {
    float point[3]; // x, y, z coordinates
    float normal[3]; // Normal vector at the point
};

class UniformSlicingAlgorithm {
public:
    UniformSlicingAlgorithm(const std::vector<Facet>& facets);
    ~UniformSlicingAlgorithm();

    void setToolLength(float toolLength);
    std::vector<float> generateSlices();
    std::vector<ContourPoint> generateContour(float z);  


private:
    std::vector<Facet> facets_;
    float toolLength_;
};

#endif // UNIFORMSLICINGALG_H
