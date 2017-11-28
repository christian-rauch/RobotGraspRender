#include <RandomObject.hpp>
#include <iostream>


RandomObject::RandomObject(const std::string obj_repo) : object_repo(obj_repo) {
    // recursively get all mesh files
    if(fs::is_directory(object_repo)) {
        for(const fs::path& p: fs::recursive_directory_iterator(object_repo)) {
            if(!fs::is_regular_file(p))
                continue;
            //std::cout << p << std::endl;
            object_mesh_list.push_back(p);
        }
    }
}

Eigen::Affine3d RandomObject::getRandomPose(const double max_transl, const double scale) {
    std::uniform_real_distribution<double> unif_angle(0.0, M_PI);

    const Eigen::AngleAxisd aa(unif_angle(generator), Eigen::Vector3d::Random().normalized());
    Eigen::Affine3d pose;
    pose.setIdentity();
    pose.linear() = aa.toRotationMatrix();
    pose.translation() = Eigen::Vector3d::Random()*max_transl;
    pose.scale(scale);
    return pose;
}

std::string RandomObject::getRandomObjectPath() {
    if(object_mesh_list.size()>0) {
        return object_mesh_list[std::uniform_int_distribution<size_t>(0, object_mesh_list.size()-1)(generator)];
    }
    else
        return "";
}

bool RandomObject::hasObjects() const {
    return object_mesh_list.size();
}
