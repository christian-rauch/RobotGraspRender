#include <string>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <lcm/lcm-cpp.hpp>
#include <robot_state_t.hpp>
#include <body_t.hpp>
#include <pose_t.hpp>

class FkPose {
public:
    FkPose(KDL::Tree &robot,
           const std::string root_frame_name,
           const std::string head_frame_name) : robot_tree(robot)
    {
        // define kinematic chain from root to target frame
        robot_tree.getChain(root_frame_name, head_frame_name, chain);
        fk = new KDL::ChainFkSolverPos_recursive(chain);
    }

    ~FkPose() {
        delete fk;
    }

    bool start() {
        if(lcm.good()) {
            lcm.subscribe("EST_ROBOT_STATE", &FkPose::handle, this);
            return true;
        }
        else {
            return false;
        }
    }

    void handle(const lcm::ReceiveBuffer* /*rbuf*/, const std::string& /*channel*/, const bot_core::robot_state_t* state) {
        // set joint names and position value
        joints.clear();
        for(uint i(0); i<uint(state->num_joints); i++) {
            joints[state->joint_name[i]] = state->joint_position[i];
        }

        // get list of joints in chain (from root to tip)
        std::vector<float> joint_values;
        for(KDL::Segment segm : chain.segments) {
            const std::string jname = segm.getJoint().getName();
            // only add values of not NONE joints
            if(segm.getJoint().getType()!=KDL::Joint::None) {
                if(joints.count(jname)) {
                    joint_values.push_back(joints.at(jname));
                }
                else {
                    // set not provided joint values to 0
                    joint_values.push_back(0);
                }
            }
        }

        assert(joint_values.size()==chain.getNrOfJoints());

        // populate chain joint values
        KDL::JntArray j(chain.getNrOfJoints());
        for(uint i(0); i<chain.getNrOfJoints(); i++) {
            j(i) = double(joint_values[i]);
        }

        // get pose of frame
        KDL::Frame frame_pose;
        fk->JntToCart(j, frame_pose);

        vicon::body_t head_pose;
        head_pose.utime = state->utime;
        head_pose.trans[0] = frame_pose.p.x();
        head_pose.trans[1] = frame_pose.p.y();
        head_pose.trans[2] = frame_pose.p.z();
        // from: KDL::Rotation -> quaternion (x,y,z,w)
        // to: vicon::body_t -> quat (w,x,y,z)
        frame_pose.M.GetQuaternion(head_pose.quat[1], head_pose.quat[2], head_pose.quat[3], head_pose.quat[0]);

        lcm.publish("BODY_TO_HEAD", &head_pose);

        bot_core::pose_t pelvis_pose;
        pelvis_pose.utime = state->utime;
        pelvis_pose.pos[0] = state->pose.translation.x;
        pelvis_pose.pos[1] = state->pose.translation.y;
        pelvis_pose.pos[2] = state->pose.translation.z;
        // bot_core::pose_t -> orientation assume (w,x,y,z)
        pelvis_pose.orientation[0] = state->pose.rotation.w;
        pelvis_pose.orientation[1] = state->pose.rotation.x;
        pelvis_pose.orientation[2] = state->pose.rotation.y;
        pelvis_pose.orientation[3] = state->pose.rotation.z;

        lcm.publish("POSE_BODY", &pelvis_pose);
    }

public:
    lcm::LCM lcm;

private:
    KDL::Tree &robot_tree;
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive *fk;
    std::map<std::string, float> joints;
};

int main(int /*argc*/, char *argv[]) {

    const std::string urdf_path = argv[1];

    const boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile(urdf_path);
    KDL::Tree robot_tree;

    if(urdf_model!=NULL) {
        kdl_parser::treeFromUrdfModel(*urdf_model, robot_tree);
    }
    else {
        std::cerr<<"no robot model"<<std::endl;
    }

    FkPose fkpose(robot_tree, "pelvis", "head");

    if(fkpose.start()==false) {
        return -1;
    }

    while(true) {
        fkpose.lcm.handle();
    }
}
