#ifndef RANDOMOBJECT_HPP
#define RANDOMOBJECT_HPP

#include <string>
#include <experimental/filesystem>
#include <random>

#include <Eigen/Eigen>

namespace fs = std::experimental::filesystem;

class RandomObject {
public:
    RandomObject(const std::string obj_repo);

    Eigen::Matrix4d getRandomPose(const double max_transl = 0, const double scale = 1);

    std::string getRandomObjectPath();

    bool hasObjects() const;

private:
    const std::string object_repo;
    std::vector<std::string> object_mesh_list;

    std::default_random_engine generator;
};

#endif // RANDOMOBJECT_HPP
