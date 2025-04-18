#include "stlfileloader.h"
#include <fstream>
#include <iostream>
#include <vector>

STLFileLoader::STLFileLoader(const std::string& filename) : filename_(filename) {}

STLFileLoader::~STLFileLoader() {}

bool STLFileLoader::loadSTLFile() {
    std::ifstream file(filename_, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename_ << std::endl;
        return false;
    }

    // Read STL file header (80 bytes)
    char header[80];
    file.read(header, 80);

    // Read number of facets (4 bytes)
    uint32_t numFacets;
    file.read(reinterpret_cast<char*>(&numFacets), 4);

    // Read facets
    facets_.clear();
    for (uint32_t i = 0; i < numFacets; ++i) {
        Facet facet;

        // Read normal vector (3 floats)
        file.read(reinterpret_cast<char*>(facet.normal), sizeof(facet.normal));

        // Read vertices (3 points, each with 3 floats)
        for (int j = 0; j < 3; ++j) {
            file.read(reinterpret_cast<char*>(facet.vertices[j]), sizeof(facet.vertices[j]));
        }

        // Skip attribute byte count (2 bytes)
        file.seekg(2, std::ios_base::cur);

        facets_.push_back(facet);
    }

    file.close();
    return true;
}

std::vector<Facet>& STLFileLoader::getFacets() {
    return facets_;
}

bool isValidSTLFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false; // File does not exist or cannot be opened
    }

    // Read STL file header (80 bytes)
    char header[80];
    file.read(header, 80);

    // Read number of facets (4 bytes)
    uint32_t numFacets;
    file.read(reinterpret_cast<char*>(&numFacets), 4);

    // Simple validation: Ensure numFacets is reasonable
    if (numFacets < 1 || numFacets > 5000000) {
        return false; // Unlikely number of facets
    }

    file.close();
    return true;
}
