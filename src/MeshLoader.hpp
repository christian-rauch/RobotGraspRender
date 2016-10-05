#ifndef MESHLOADER_HPP
#define MESHLOADER_HPP

#include <string>
#include <vector>
#include <array>
#include <memory>

#include <Eigen/Core>

typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Point3DList;
typedef Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> Point2DList;
typedef Eigen::Matrix<uint, Eigen::Dynamic, 3, Eigen::RowMajor> Index3DList;

class Mesh {
public:
    Point3DList vertices;
    Point3DList normals;
    Point2DList texture;
    Point3DList colour; // r, g, b, no alpha
    Index3DList faces;

    Mesh(const uint nVerts, const uint nFaces) {
        vertices.resize(nVerts, 3);
        normals.resize(nVerts, 3);
        texture.resize(nVerts, 2);
        colour.resize(nVerts, 3);
        faces.resize(nFaces, 3);
    }

    const float *getVertices() const { return vertices.data(); }

    const uint *getFaces() const { return faces.data(); }
};

typedef std::shared_ptr<Mesh> MeshPtr;

class MeshLoader {
public:
    MeshLoader();

    static std::vector<MeshPtr> getMesh(const std::string &path);
};


#endif // MESHLOADER_HPP
