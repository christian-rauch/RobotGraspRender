#ifndef MESHLOADER_HPP
#define MESHLOADER_HPP

#include <string>
#include <vector>
#include <array>
#include <memory>

#include <pangolin/image/image_io.h>

//#include <Eigen/Core>

//typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Point3DList;
//typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> Point2DList;
//typedef Eigen::Matrix<uint, Eigen::Dynamic, 3, Eigen::RowMajor> Index3DList;

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
};

typedef std::shared_ptr<Mesh> MeshPtr;

namespace MeshLoader {

MeshPtr getMesh(const std::string &path);

}


#endif // MESHLOADER_HPP
