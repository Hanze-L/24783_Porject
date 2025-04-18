#ifndef STLFLELOADER_H
#define STLFLELOADER_H

#include <vector>
#include <string>

struct Facet {
    float normal[3];
    float vertices[3][3];
};

class STLFileLoader {
public:
    // Add default parameter to make this a default constructor
    STLFileLoader(const std::string& filename = "");  
    ~STLFileLoader();

    bool loadSTLFile();
    std::vector<Facet>& getFacets();

    void setFilename(const std::string& filename) { filename_ = filename; }

private:
    std::string filename_;
    std::vector<Facet> facets_;
};

bool isValidSTLFile(const std::string& filename);

#endif // STLFLELOADER_H
