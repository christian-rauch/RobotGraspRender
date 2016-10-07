#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <urdf_model/model.h>
#include <kdl/tree.hpp>

#include "Mesh.hpp"
#include "MeshLoader.hpp"

typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePrt;

class RobotModel {
private:
    ModelInterfacePrt urdf_model;
    KDL::Tree robot_tree;

    std::string mesh_package_path;

public:
    RobotModel() { }

    RobotModel(const std::string &urdf_path);

    void parseURDF(const std::string &urdf_path);

    void loadLinkMeshes();

    void renderSetup();

    void render(pangolin::GlSlProgram &shader);

    std::map<std::string, MeshPtr> link_meshes;
};

#endif // ROBOTMODEL_HPP
