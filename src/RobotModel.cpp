#include "RobotModel.hpp"

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser.hpp>

#include <boost/filesystem.hpp>

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

                std::cout<<"link: "<<l->name<<" has mesh: "<<mesh_path<<std::endl;

                link_meshes[l->name] = MeshLoader::getMesh(mesh_path);
            }
        }
    }
}

void RobotModel::renderSetup() {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        it->second->renderSetup();
    }
}

void RobotModel::render(pangolin::GlSlProgram &shader) {
    for(auto it = link_meshes.begin(); it!=link_meshes.end(); it++) {
        it->second->render(shader);
    }
}
