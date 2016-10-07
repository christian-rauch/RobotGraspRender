#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <array>

#include <pangolin/image/image_io.h>

typedef std::vector< std::array<float, 3> > Point3DList;
typedef std::vector< std::array<float, 2> > Point2DList;
typedef std::vector< std::array<uint, 3> > Index3DList;

struct Mesh {
    Point3DList vertices;
    Point3DList normals;
    Point2DList uv;     // U,V coordinates
    Point3DList colour; // r, g, b, no alpha
    Index3DList faces;

    std::string directory;

    pangolin::TypedImage texture; // only a single material for the whole scene for now

    bool hasFaces() { return faces.size()!=0; }

    bool hasColour() { return colour.size()!=0; }

    bool hasTexture() { return uv.size()!=0; }

    void renderMeshTexture();
};

#endif // MESH_H
