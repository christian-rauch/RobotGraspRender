#include "RobotModel.hpp"

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser.hpp>

#include <boost/filesystem.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>

#define PACKAGE_PATH_URI_SCHEME "package://"

// find the file "package.xml" for obtaining the root folder of our mesh
std::string getROSPackagePath(const std::string &start_path,
                              const std::string ros_package_path_file = "package.xml")
{
    boost::filesystem::path fpath = boost::filesystem::canonical(start_path);

    // search backwards for package path, e.g. directory that contains the package file
    while(fpath.has_parent_path() && !boost::filesystem::is_regular_file(fpath / ros_package_path_file)) {
        // go one step backward closer to root
        fpath = fpath.parent_path();
    }

    std::string package_path = "";
    if(!boost::filesystem::is_regular_file(fpath / ros_package_path_file)) {
        // package path not found, use relative path
        package_path = boost::filesystem::canonical(start_path).parent_path().native()
                + boost::filesystem::path::preferred_separator;
    }
    else {
        // store package path with trailing directory seperator
        package_path = fpath.branch_path().native() + boost::filesystem::path::preferred_separator;
    }
    return package_path;
}

RobotModel::RobotModel(const std::string &urdf_path) {
    parseURDF(urdf_path);
}

void RobotModel::parseURDF(const std::string &urdf_path) {
    // read URDF and create kinematic tree
    urdf_model = urdf::parseURDFFile(urdf_path);
    if(urdf_model!=NULL) {
        kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree);
    }
    else {
        std::cerr<<"no robot model"<<std::endl;
    }

    // get location of urdf package for mesh path
    std::string urdf_dir = urdf_path.substr(0, urdf_path.find_last_of('/'));
    mesh_package_path = getROSPackagePath(urdf_dir);
}

void RobotModel::loadLinkMeshes() {

    // get root frame
    root_frame = urdf_model->getRoot()->name;

    std::vector<boost::shared_ptr<urdf::Link>> links;
    urdf_model->getLinks(links);

    // load mesh for each link
    for(boost::shared_ptr<urdf::Link> l : links) {
        if(l->visual!=NULL) {
            //
            if(l->visual->geometry->type==urdf::Geometry::MESH) {
                std::string mesh_path = dynamic_cast<urdf::Mesh*>(&*l->visual->geometry)->filename;
//                std::cout<<"link: "<<l->name<<" has mesh: "<<mesh_path<<std::endl;

                if(mesh_path.find(PACKAGE_PATH_URI_SCHEME) != std::string::npos) {
                    // we need to replace "package://" by full path
                    boost::algorithm::replace_first(mesh_path, PACKAGE_PATH_URI_SCHEME, mesh_package_path);
                }
                else {
                    // prepend full path
                    mesh_path = mesh_package_path + mesh_path;
                }

                //std::cout<<"link: "<<l->name<<" has mesh: "<<mesh_path<<std::endl;

                link_meshes[l->name] = MeshLoader::getMesh(mesh_path);
            }
        }
    }
}

void RobotModel::loadJointNames() {
    T_wr.SetIdentity();
    // initialise joint map
    for(auto j : urdf_model->joints_) {
        joints[j.first] = 0;
    }
}

void RobotModel::renderSetup() {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        it->second->renderSetup();
    }
}

pangolin::OpenGlMatrix RobotModel::getFramePose(const std::string frame) {
    // kineamtic chain from root to frame
    KDL::Chain chain;
    robot_tree.getChain(root_frame, frame, chain);
    KDL::ChainFkSolverPos_recursive fk(chain);

    // get list of joints in chain (from root to tip)
    std::vector<float> joint_values;
    for(KDL::Segment segm : chain.segments) {
        const std::string jname = segm.getJoint().getName();
        // only add values of not NONE joints
        if(segm.getJoint().getType()!=KDL::Joint::None) {
            joint_values.push_back(joints.at(jname));
        }
    }

    assert(joint_values.size()==chain.getNrOfJoints());

    // populate chain joint values
    KDL::JntArray j(chain.getNrOfJoints());
    for(uint i(0); i<chain.getNrOfJoints(); i++) {
        j(i) = joint_values[i];
    }

    // get pose of frame
    KDL::Frame frame_pose;
    fk.JntToCart(j, frame_pose);

    // model matrix
    pangolin::OpenGlMatrix M;
    M.SetIdentity();
    // translation
    M(0, 3) = frame_pose.p.x();
    M(1, 3) = frame_pose.p.y();
    M(2, 3) = frame_pose.p.z();
    // rotation
    M(0,0) = frame_pose.M(0,0);
    M(0,1) = frame_pose.M(0,1);
    M(0,2) = frame_pose.M(0,2);
    M(1,0) = frame_pose.M(1,0);
    M(1,1) = frame_pose.M(1,1);
    M(1,2) = frame_pose.M(1,2);
    M(2,0) = frame_pose.M(2,0);
    M(2,1) = frame_pose.M(2,1);
    M(2,2) = frame_pose.M(2,2);

    return M;
}

void RobotModel::render(pangolin::GlSlProgram &shader) {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {

        // skip meshes that should not be rendered
        if(skipMeshes.count(it->first))
            continue;

        pangolin::OpenGlMatrix M = getFramePose(it->first);

        // apply frame transformation to shader
        shader.Bind();
        shader.SetUniform("M", T_wr*M);
        shader.Unbind();

        it->second->render(shader);
    }
}
