#include <pangolin/pangolin.h>

#include <MeshLoader.hpp>

#include <RobotModel.hpp>

#include <lcm/lcm-cpp.hpp>
#include <robot_state_t.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

class LcmRobotState {
public:
    std::map<std::string, float> joints;

    pangolin::OpenGlMatrix T_wr;

    LcmRobotState() {
        T_wr.SetIdentity();
    }

    void onRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg) {
        // joints
        for(uint i(0); i<msg->num_joints; i++) {
            joints[msg->joint_name[i]] = msg->joint_position[i];
        }

        // robot pose
        T_wr(0, 3) = msg->pose.translation.x;
        T_wr(1, 3) = msg->pose.translation.y;
        T_wr(2, 3) = msg->pose.translation.z;
        // rotation from quaterion
        const Eigen::Matrix3f rot =
                Eigen::Quaternion<float>(msg->pose.rotation.w, msg->pose.rotation.x,
                                         msg->pose.rotation.y, msg->pose.rotation.z)
                .toRotationMatrix();
        T_wr(0,0) = rot(0,0);
        T_wr(0,1) = rot(0,1);
        T_wr(0,2) = rot(0,2);
        T_wr(1,0) = rot(1,0);
        T_wr(1,1) = rot(1,1);
        T_wr(1,2) = rot(1,2);
        T_wr(2,0) = rot(2,0);
        T_wr(2,1) = rot(2,1);
        T_wr(2,2) = rot(2,2);
    }
};

int main(int argc, char *argv[]) {
    ////////////////////////////////////////////////////////////////////////////
    /// Configuration
    pangolin::ParseVarsFile(argv[1]);

    pangolin::Var<std::string> lcm_channel("lcm_channel");
    pangolin::Var<std::string> env_path("environment_mesh");
    pangolin::Var<std::string> obj_path("object_mesh");
    pangolin::Var<std::string> robot_model_path("robot_model");

    std::cout<<"channel: "<<lcm_channel<<std::endl;
    std::cout<<"robot: "<<robot_model_path<<std::endl;
    std::cout<<"environment: "<<env_path<<std::endl;
    std::cout<<"object: "<<obj_path<<std::endl;

    ////////////////////////////////////////////////////////////////////////////
    /// LCM Setup
    LcmRobotState lrs;
    lcm::LCM lcm;
    lcm.subscribe(lcm_channel, &LcmRobotState::onRobotState, &lrs);

    ////////////////////////////////////////////////////////////////////////////
    /// Window Setup
    pangolin::CreateWindowAndBind("RobotGraspRender",640,480);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
      pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
    );

    // robot view
    // multisense paramters
    pangolin::OpenGlRenderState robot_view(
      pangolin::ProjectionMatrix(1024, 1024,  // width x height
                                 556.183166504, // f_u
                                 556.183166504, //f_v
                                 512, 512,      // centre coordinates
                                 0.0001,1000)
    );

//    pangolin::View& d_cam = pangolin::CreateDisplay()
//      .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
//      .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View &d_cam = pangolin::Display("free view")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View &robot_cam = pangolin::Display("robot view")
            .SetAspect(640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(robot_view));

    pangolin::Display("multi")
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutEqual)
          .AddDisplay(d_cam)
          .AddDisplay(robot_cam);

    // load and compile shaders
    pangolin::GlSlProgram prog;
    prog.AddShaderFromFile(pangolin::GlSlVertexShader, "SimpleVertexShader.vert");
    prog.AddShaderFromFile(pangolin::GlSlFragmentShader, "SimpleFragmentShader.frag");
    prog.Link();

    pangolin::GlSlProgram texture_shader;
    texture_shader.AddShaderFromFile(pangolin::GlSlVertexShader, "TextureVertexShader.vert");
    texture_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, "TextureFragmentShader.frag");
    texture_shader.Link();

    ////////////////////////////////////////////////////////////////////////////
    /// Load Resources

    RobotModel robot;
    if(!robot_model_path->empty()) {
        robot.parseURDF(robot_model_path);
        robot.loadLinkMeshes();
        robot.loadJointNames();
    }

    // meshes
    MeshPtr env;
    if(env_path->empty()) {
        env = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        env = MeshLoader::getMesh(env_path);
    }
    std::cout<<"env verts: "<<env->vertices.size()<<std::endl;
    std::cout<<"env faces: "<<env->faces.size()<<std::endl;

    MeshPtr obj;
    if(obj_path->empty()) {
        obj = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        obj = MeshLoader::getMesh(obj_path);
    }
    std::cout<<"obj verts: "<<obj->vertices.size()<<std::endl;
    std::cout<<"obj faces: "<<obj->faces.size()<<std::endl;

    // setup opengl buffers for meshes
    env->renderSetup();
    obj->renderSetup();
    robot.renderSetup();

    ////////////////////////////////////////////////////////////////////////////
    /// Draw

    while(!pangolin::ShouldQuit()) {

        lcm.handleTimeout(10);

        // clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

//        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1);

        prog.Bind();
        prog.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        pangolin::OpenGlMatrix I;
        I.SetIdentity();
        prog.SetUniform("M", I);
        prog.Unbind();
        texture_shader.Bind();
        texture_shader.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        texture_shader.Unbind();

        // render environment
        env->render(prog);

        // articulate and render robot
        robot.T_wr = lrs.T_wr;
        for(auto kv : lrs.joints) {
            robot.joints[kv.first] = kv.second;
        }
        robot.render(texture_shader);

        // get hand pose from robot and render object at this pose
        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePose("l_hand_face"));
        prog.Unbind();
        obj->render(prog);

        /// robot view
        robot_cam.Activate(robot_view);

        // coordinates in camera frame: look from origin in Z-direction
        robot_view.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,0,0,0,3,pangolin::AxisY));

        // follow relative to camera motion
        const pangolin::OpenGlMatrix cam_frame = robot.T_wr*robot.getFramePose("left_camera_optical_frame");
        robot_view.Follow(cam_frame.Inverse());

        pangolin::glDrawAxis(0.5);

        prog.Bind();
        prog.SetUniform("MVP", robot_view.GetProjectionModelViewMatrix());
        I.SetIdentity();
        prog.SetUniform("M", I);
        prog.Unbind();
        texture_shader.Bind();
        texture_shader.SetUniform("MVP", robot_view.GetProjectionModelViewMatrix());
        texture_shader.Unbind();

        env->render(prog);

        robot.addSkip("upperNeckPitchLink");
        robot.render(texture_shader);
        robot.resetSkip();

        robot_view.Unfollow();

        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePose("l_hand_face"));
        prog.Unbind();
        obj->render(prog);

        // draw
        pangolin::FinishFrame();
    }

    return 0;
}
