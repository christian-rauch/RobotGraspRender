#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <urdf_model/model.h>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include <set>

#include "Mesh.hpp"
#include "MeshLoader.hpp"

typedef boost::shared_ptr<urdf::ModelInterface> ModelInterfacePrt;

class RobotModel {
private:
    ModelInterfacePrt urdf_model;

    KDL::Tree robot_tree;

    std::string root_frame;

    std::string mesh_package_path;

    std::set<std::string> skipMeshes;

public:
    RobotModel() { }

    RobotModel(const std::string &urdf_path);

    void parseURDF(const std::string &urdf_path);

    /**
     * @brief loadLinkMeshes load all meshes defined in URDF
     */
    void loadLinkMeshes();

    void voxelize(const float resolution=0.01, const float precision=0.01);

    /**
     * @brief loadJointNames initialise the joints and the robot pose
     */
    void loadJointNames();

    /**
     * @brief renderSetup initialise OpenGL buffer and upload mesh data
     */
    void renderSetup();

    void generateMeshColours(const bool single_colour=true, const bool gray=false);

    void render(pangolin::GlSlProgram &shader);

    void addSkip(const std::string& link) { skipMeshes.insert(link); }

    void resetSkip() { skipMeshes.clear(); }

    pangolin::OpenGlMatrix getFramePose(const std::string frame);

    std::map<std::string, MeshPtr> link_meshes;

    std::map<std::string, pangolin::Colour> link_colours;

    std::map<std::string, float> joints;    // current joint positions

    pangolin::OpenGlMatrix T_wr;    // robot pose in world
};

#endif // ROBOTMODEL_HPP
