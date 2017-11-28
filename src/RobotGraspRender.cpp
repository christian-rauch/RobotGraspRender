#include <pangolin/pangolin.h>

#include <MeshLoader.hpp>
#include <RobotModel.hpp>
#include <RandomObject.hpp>

#include <lcm/lcm-cpp.hpp>
#include <robot_state_t.hpp>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <ios>

#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

class LcmRobotState {
public:
    std::map<std::string, float> joints;
    int64_t time;

    pangolin::OpenGlMatrix T_wr;

    LcmRobotState() {
        T_wr.SetIdentity();
    }

    void onRobotState(const lcm::ReceiveBuffer* /*rbuf*/, const std::string& /*channel*/, const bot_core::robot_state_t* msg) {
        setRobotState(msg);
    }

    void setRobotState(const bot_core::robot_state_t* msg) {
        // time stamp in microseconds
        time = msg->utime;

        // joints
        for(uint i(0); i<uint(msg->num_joints); i++) {
            joints[msg->joint_name[i]] = msg->joint_position[i];
        }

        // robot pose
        T_wr(0, 3) = msg->pose.translation.x;
        T_wr(1, 3) = msg->pose.translation.y;
        T_wr(2, 3) = msg->pose.translation.z;
        // rotation from quaterion
        const Eigen::Matrix3d rot =
                Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x,
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

class CSVjoints {
public:
    CSVjoints() { }

    ~CSVjoints() {
        csv_file.close();
    }

    /**
     * @brief removeWhiteSpaces, remove ' ', '\t', '\n', '\v', '\f', '\r' from string
     * @param joint_name string from which white spaces will be removed
     */
    static void removeWhiteSpaces(std::string &joint_name) {
        joint_name.erase(std::remove_if(joint_name.begin(), joint_name.end(), [](char x){return std::isspace(x);}), joint_name.end());
    }

    bool open(const std::string &joints_path, const std::string &timestamps_path) {
        csv_file.open(joints_path);
        if(!csv_file.good())
            std::cerr<<"error openning joints file: "<<joints_path<<std::endl;

        if(!timestamps_path.empty()) {
            timestamps_file.open(timestamps_path);
            if(!timestamps_file.is_open()) {
                std::cerr<<"error openning timestamps file: "<<timestamps_path<<std::endl;
                return false;
            }
        }
        else {
            std::cout<<"ignoring timestamps"<<std::endl;
        }

        if(csv_file.fail())
            return false;

        return isOpen();
    }

    bool isOpen() {
        return csv_file.is_open();
    }

    void setJointNames() {
        jnames.clear();
        std::string jname_list;

        // joint names in first line of csv file
        std::getline(csv_file, jname_list);

        std::istringstream iss(jname_list);
        std::string joint_name;
        while (std::getline(iss, joint_name, ' ')) {
            if(!joint_name.empty()) {
                removeWhiteSpaces(joint_name);
                jnames.push_back(joint_name);
            }
        }
    }

    // fetch next line of joint values
    std::tuple<std::map<std::string, float>, int64_t> getNext() {
        if(jnames.size()==0)
            throw std::runtime_error("joint names not defined");

        // joint position values
        std::vector<float> jvalues;
        std::string val_line;
        if(std::getline(csv_file, val_line).good()) {
            // continue
            std::stringstream ss_line(val_line);
            std::string v;
            while(std::getline(ss_line, v, ' ')) {
                jvalues.push_back(std::stof(v));
            }
        }

        // timestamps
        std::string time_line;
        int64_t timestamp;
        if(std::getline(timestamps_file, time_line).good()) {
            timestamp = std::stol(time_line);
        }
        else {
            timestamp = 0;
        }

        std::map<std::string, float> joints;
        if(jnames.size()==jvalues.size()) {
            // create named joints
            std::transform(jnames.begin(), jnames.end(), jvalues.begin(),
                   std::inserter(joints, joints.end()), std::make_pair<std::string const&, float const&>);
        }
        else if(jvalues.size()==0) {
            std::cerr<<"end of file"<<std::endl;
        }
        else {
            throw std::runtime_error("joint size does not match");
        }
        return std::make_tuple(joints, timestamp);
    }

private:
    std::ifstream csv_file;
    std::ifstream timestamps_file;
    std::vector<std::string> jnames;
};

// conversion from string to matrix
template<typename T>
void deserialise_matrix(const std::string &mat_string, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &M) {
    std::stringstream mat_ss(mat_string);
    std::string row;
    uint irow = 0;
    while(std::getline(mat_ss,row,',')) {
        M.conservativeResize(irow+1, Eigen::NoChange);
        std::stringstream row_ss(row);
        uint icol = 0;
        std::string v;
        while(std::getline(row_ss,v, ' ')) {
            if(v.size()!=0) {
                if(irow==0) {
                    M.conservativeResize(Eigen::NoChange, icol+1);
                }
                std::stringstream(v) >> M(irow,icol);
                icol++;
            }
        }
        irow++;
    }
}

int main(int /*argc*/, char *argv[]) {
    ////////////////////////////////////////////////////////////////////////////
    /// Configuration file
    pangolin::ParseVarsFile(argv[1]);

    // data source parameters
    const pangolin::Var<std::string> lcm_channel("lcm_channel");
    const pangolin::Var<std::string> env_path("environment_mesh");
    const pangolin::Var<std::string> obj_path("object_mesh");
    const pangolin::Var<std::string> robot_model_path("robot_model");
    const fs::path export_dir = pangolin::Var<std::string>("dest_dir").Get();
    const pangolin::Var<uint> nframes("save_nframes");
    const pangolin::Var<std::string> logfile("log_file");
    const pangolin::Var<std::string> joint_conf_path("joint_config_path");
    const pangolin::Var<std::string> timestamps_path("timestamps_path");

    const pangolin::Var<std::string> grasp_frame("grasp_frame");

    const pangolin::Var<std::string> object_repo("object_repo");

    const pangolin::Var<std::string> opose_string("object_pose");
    pangolin::OpenGlMatrix object_pose;
    if(std::string(opose_string).size()>0) {
        Eigen::MatrixXf mx;
        deserialise_matrix<float>(opose_string, mx);
        const Eigen::Matrix4f m4(mx);
        object_pose = pangolin::OpenGlMatrix(m4);
    }
    else {
        object_pose.SetIdentity();
    }
    const pangolin::Var<bool> object_occlusion("object_occlusion");

    // camera parameters
    const pangolin::Var<std::string> camera_frame("camera_frame");
    const pangolin::Var<uint> cam_width("width");
    const pangolin::Var<uint> cam_height("height");
    const pangolin::Var<uint> centre_x("centre_x");
    const pangolin::Var<uint> centre_y("centre_y");
    const pangolin::Var<double> f_u("f_u");
    const pangolin::Var<double> f_v("f_v");
    const pangolin::Var<double> rotate_z_rad("rotate_z_rad");

    const pangolin::Var<std::string> cpose_string("camera_pose");
    pangolin::OpenGlMatrix camera_pose;
    Eigen::Isometry3f cam_pose_iso3;
    if(std::string(cpose_string).size()>0) {
        Eigen::MatrixXf mx;
        deserialise_matrix<float>(cpose_string, mx);
        const Eigen::Matrix4f m4(mx);
        cam_pose_iso3 = m4;
        camera_pose = pangolin::OpenGlMatrix(m4);
    }

    // export flags
    const pangolin::Var<bool> save_background("save_background");
    const pangolin::Var<bool> save_object("save_object");
    const pangolin::Var<bool> save_robot("save_robot");

    const pangolin::Var<bool> export_colour("export_colour");
    const pangolin::Var<bool> export_depth_viz("export_depth_viz");
    const pangolin::Var<bool> export_depth("export_depth");
    const pangolin::Var<bool> export_label("export_label");
    const pangolin::Var<bool> export_part_masks("export_part_masks");
    const pangolin::Var<bool> label_gray("label_gray");

    const pangolin::Var<std::string> export_link_poses("export_link_poses");

    std::cout<<"channel: "<<lcm_channel.Get()<<std::endl;
    std::cout<<"robot: "<<robot_model_path.Get()<<std::endl;
    std::cout<<"environment: "<<env_path.Get()<<std::endl;
    std::cout<<"object: "<<obj_path.Get()<<std::endl;
    std::cout<<"save images every "<<nframes<<" frames to: "<<export_dir<<std::endl;

    ////////////////////////////////////////////////////////////////////////////
    /// I/O Setup
    LcmRobotState lrs;
    lcm::LCM *lcm = NULL;
    lcm::LogFile *log = NULL;
    CSVjoints csvj;

    // chose from 3 joint value providers:
    // - CSV file
    // - LCM log file
    // - LCM live

    if(!std::string(joint_conf_path).empty()) {
        // read joints values from csv and quit at end-of-file
        std::cout<<"reading joint config from: "<<joint_conf_path.Get()<<std::endl;
        if(!csvj.open(joint_conf_path, timestamps_path))
            std::cerr<<"could not open files"<<std::endl;
        csvj.setJointNames();
    }
    else {
        // use LCM as joint value provider
        if(!std::string(logfile).empty()) {
            // read from LCM logfile
            std::cout<<"reading from log file: "<<logfile.Get()<<std::endl;
            log = new lcm::LogFile(std::string(logfile), "r");
            if(!log->good()) {
                std::cerr << "error: reading log file" << std::endl;
            }
        }
        else {
            // read from live LCM
            lcm = new lcm::LCM();
            if(!lcm->good()) {
                std::cerr << "error: creating LCM" << std::endl;
            }
            else {
                if(!std::string(lcm_channel).empty()) {
                    lcm->subscribe(lcm_channel, &LcmRobotState::onRobotState, &lrs);
                }
                else {
                    std::cerr << "no LCM channel provided" << std::endl;
                }
            }
        }
    }

    // check and create target directories
    std::map<std::string, fs::path> export_dirs; // name, path
    if(export_colour)
        export_dirs["colour"] = export_dir / "colour";
    if(export_depth_viz)
        export_dirs["depth_viz"] = export_dir / "depth_viz";
    if(export_depth)
        export_dirs["depth"] = export_dir / "depth";
    if(export_label)
        export_dirs["label"] = export_dir / "label";
    if(export_part_masks)
        export_dirs["masks"] = export_dir / "masks";
    export_dirs["joint_pos"] = export_dir / "joint_pos";

    for(const auto& dir : export_dirs) {
        fs::create_directories(dir.second);
    }

    // create file for timestamps
    std::ofstream joint_time(export_dir / "time.csv");

    RandomObject rand_obj(object_repo);

    ////////////////////////////////////////////////////////////////////////////
    /// Window Setup
    pangolin::CreateWindowAndBind("RobotGraspRender",640,480);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
      pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
    );

    // robot camera
    const uint w = cam_width;
    const uint h = cam_height;
    const double z_near = 0.0001;
    const double z_far = 1000;
    pangolin::OpenGlRenderState robot_cam(
      pangolin::ProjectionMatrix(w, h,  // width x height
                                 f_u, // f_u
                                 f_v, // f_v
                                 centre_x, centre_y,      // centre coordinates
                                 z_near, z_far)
    );

    // replace invalid depth values
    const double depth_cutoff = 20; // meter
    const uint depth_max_export_mm = 16384; // mm

    pangolin::View &d_cam = pangolin::Display("free view")
            .SetAspect(640.0/480.0)
            .SetHandler(new pangolin::Handler3D(s_cam));
    pangolin::View &robot_view = pangolin::Display("robot view")
            .SetAspect(w/double(h))
            .SetHandler(new pangolin::Handler3D(robot_cam));
    pangolin::View &depth_view = pangolin::Display("depth")
        .SetAspect(w/double(h))
        .SetHandler(new pangolin::Handler3D(robot_cam));
    pangolin::View &label_view = pangolin::Display("labels")
        .SetAspect(w/double(h))
        .SetHandler(new pangolin::Handler3D(robot_cam));

    pangolin::Display("multi")
          .SetBounds(0.0, 1.0, 0.0, 1.0)
          .SetLayout(pangolin::LayoutEqual)
          .AddDisplay(d_cam)
          .AddDisplay(robot_view)
          .AddDisplay(depth_view)
          .AddDisplay(label_view);

    ////////////////////////////////////////////////////////////////////////////
    /// Load Resources

    // robot
    RobotModel robot;
    if(!robot_model_path.Get().empty()) {
        robot.parseURDF(robot_model_path);
        // load meshes and set unique colour
        robot.loadLinkMeshes();
        // initialising joints and pose
        robot.loadJointNames();
        robot.camera_frame_name = camera_frame;
        robot.generateMeshColours(false, label_gray);
    }

    // export link names and labels
    if(robot.link_label_colours.size()!=robot.link_label_id.size())
        throw std::runtime_error("link label and colour size mismatch");

    std::ofstream link_label_file(export_dir / "link_label.csv");
    //link_label_file << "name id r g b"<<std::endl;
    for(auto it = robot.link_meshes.begin(); it!=robot.link_meshes.end(); it++) {
        const std::string link_name = it->first;
        // link name and id (gray value)
        link_label_file << link_name << " " << robot.link_label_id[link_name];
        // colour channels
        link_label_file << " " << robot.link_label_colours_rgb[link_name].r << " " <<robot.link_label_colours_rgb[link_name].g << " " << robot.link_label_colours_rgb[link_name].b;
        link_label_file << std::endl;
    }
    link_label_file.close();

    // read link names for which to export pose
    std::vector<std::string> export_link_pose_names;
    if(export_link_poses.Get() == "all") {
        // export all links
        for(const auto& li : robot.link_label_id) {
            export_link_pose_names.push_back(li.first);
        }
    }
    else {
        std::istringstream iss(export_link_poses);
        std::string link_name;
        while (std::getline(iss, link_name, ',')) {
            // remove whitespace
            link_name.erase(remove_if(link_name.begin(), link_name.end(), isspace), link_name.end());
            export_link_pose_names.push_back(link_name);
        }
    }

    // create files for pose export and write header
    const fs::path pose_path = export_dir / "frame_pose";
    fs::create_directories(pose_path);
    std::map<std::string, std::shared_ptr<std::ofstream>> pose_export_files;
    for(const std::string& link : export_link_pose_names) {
        pose_export_files[link] = std::make_shared<std::ofstream>(pose_path/fs::path(link+"_pose.csv"));
        (*pose_export_files[link]) << "px py pz qw qx qy qz" << std::endl;
    }

    // create files for joint 2D position export and write header
    std::map<std::string, std::shared_ptr<std::ofstream>> joint_pos_export_files;
    for(const std::string& link : export_link_pose_names) {
        joint_pos_export_files[link] = std::make_shared<std::ofstream>(export_dirs.at("joint_pos") / (link+"_2Dpos.csv"));
        (*joint_pos_export_files[link]) << "x y" << std::endl;
    }


    // meshes
    MeshPtr env;
    if(env_path.Get().empty()) {
        env = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        env = MeshLoader::getMesh(env_path);
    }
//    std::cout<<"env verts: "<<env->vertices.size()<<std::endl;
//    std::cout<<"env faces: "<<env->faces.size()<<std::endl;

    MeshPtr obj;
    if(obj_path.Get().empty()) {
        obj = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        obj = MeshLoader::getMesh(obj_path);
    }
//    std::cout<<"obj verts: "<<obj->vertices.size()<<std::endl;
//    std::cout<<"obj faces: "<<obj->faces.size()<<std::endl;

    ////////////////////////////////////////////////////////////////////////////
    /// OpenGL Setup (Shader, Buffer)

    // load and compile shaders
    pangolin::GlSlProgram prog;
    prog.AddShaderFromFile(pangolin::GlSlVertexShader, "SimpleVertexShader.vert");
    prog.AddShaderFromFile(pangolin::GlSlFragmentShader, "SimpleFragmentShader.frag");
    prog.Link();

    pangolin::GlSlProgram texture_shader;
    texture_shader.AddShaderFromFile(pangolin::GlSlVertexShader, "TextureVertexShader.vert");
    texture_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, "TextureFragmentShader.frag");
    texture_shader.Link();

    pangolin::GlSlProgram label_shader;
    label_shader.AddShaderFromFile(pangolin::GlSlVertexShader, "LabelVertexShader.vert");
    label_shader.AddShaderFromFile(pangolin::GlSlFragmentShader, "LabelFragmentShader.frag");
    label_shader.Link();

    // setup opengl buffers for meshes
    env->renderSetup();
    obj->renderSetup();
    robot.renderSetup();

    // off-screen buffer
    pangolin::GlTexture color_buffer(w,h);
    pangolin::GlRenderBuffer depth_buffer(w,h);
    pangolin::GlFramebuffer fbo_buffer(color_buffer, depth_buffer);

    std::vector<float> depth_data(w*h);
    std::vector<uint16_t> depth_data_mm(w*h); // depth in mm
    std::vector<uint8_t> depth_data_vis(w*h);

    ////////////////////////////////////////////////////////////////////////////
    /// Draw
    uint iframe = 0;
    uint iimg = 0;
    bool store_img = false;

    while(!pangolin::ShouldQuit()) {

        iframe++;

        // store img every n frames
        store_img = (nframes>0) ? (iframe%nframes == 0) : false;

        if(store_img)
            iimg++;

        // handle messages
        if(lcm!=NULL) {
            // read live from channel
            lcm->handleTimeout(10);
        }

        // read event from log file
        if(log!=NULL) {
            const lcm::LogEvent *event = NULL;
            // read log until event with joint channel
            do {
                event = log->readNextEvent();
            } while(event!=NULL && event->channel!=std::string(lcm_channel));

            if(event) {
                bot_core::robot_state_t state;
                state.decode(event->data, 0, event->datalen);
                lrs.setRobotState(&state);
            }
            else {
                std::cout << "end of log" << std::endl;
                break;
            }
        }

        // clear colour and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // set background colour
//        glClearColor(0.5, 0.5, 0.5, 1);

        /// free view
        d_cam.Activate(s_cam);

        // world frame
        pangolin::glDrawAxis(1);

        // camera frame
        pangolin::glDrawAxis(camera_pose, 0.3);

        prog.Bind();
        prog.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        pangolin::OpenGlMatrix I;
        I.SetIdentity();
        prog.SetUniform("M", I);
        prog.Unbind();
        texture_shader.Bind();
        texture_shader.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        texture_shader.Unbind();

        label_shader.Bind();
        label_shader.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        label_shader.Unbind();

        // render environment
        env->render(prog);

        // articulate robot
        robot.T_wr = lrs.T_wr;
        if(lcm!=NULL || log!=NULL) {
            joint_time << lrs.time << std::endl;
            for(auto kv : lrs.joints) {
                robot.joints[kv.first] = kv.second;
            }
        }

        if(csvj.isOpen()) {
            std::map<std::string, float> jv;
            int64_t time;
            std::tie(jv, time) = csvj.getNext();
            if(!jv.empty()) {
                robot.joints = jv;
                joint_time << time << std::endl;
            }
            else {
                // end of CSV file
                break;
            }
        }

        // update link/mesh poses
        robot.updateFrames();

        // random object and pose in hand frame
        if(rand_obj.hasObjects()) {
            object_pose = rand_obj.getRandomPose(0.0);
            const std::string rand_obj_path = rand_obj.getRandomObjectPath();
            //std::cout << iframe << " " << op << std::endl;
            obj = MeshLoader::getMesh(rand_obj_path);
            obj->renderSetup();
        }
        if(object_occlusion) {
            // render object in the line of sight to grasp frame
            object_pose = rand_obj.getRandomPose(0.1); // T_po
            const Eigen::Affine3d T_wp(robot.T_wr*robot.getFramePoseMatrix(grasp_frame));
            // palm pose in camera frame
            const Eigen::Affine3d T_cp(camera_pose.Inverse() * T_wp);
            // target object position in camera frame
            Eigen::Affine3d T_co = T_cp * Eigen::Affine3d(object_pose);
            T_co.translation() *= 1-0.2;
            object_pose = T_wp.inverse() * Eigen::Affine3d(camera_pose) * T_co;
        }

        // show coordinate axis of exported frames
        for(const std::string& link : export_link_pose_names) {
            const pangolin::OpenGlMatrix frame_pose = robot.T_wr*robot.getFramePoseMatrix(link);
            pangolin::glDrawAxis(frame_pose, 0.3);
        }

        // render actual robot
        robot.render(label_shader, true);

        const pangolin::OpenGlMatrix T_po = object_pose;

        // get hand pose from robot and render object at this pose
        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
        prog.Unbind();
        obj->render(prog);

        /// robot view
        robot_view.Activate(robot_cam);

        // coordinates in camera frame: look from origin in Z-direction
        robot_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,0,0,0,3,pangolin::AxisY));

        // set camera frame from robot kinematic chain if not provided from variable 'camera_pose'
        if(!(std::string(cpose_string).size()>0)) {
            camera_pose = robot.T_wr*robot.getFramePoseMatrix(robot.camera_frame_name);
        }
        const pangolin::OpenGlMatrix cam_frame = camera_pose*pangolin::OpenGlMatrix::RotateZ(rotate_z_rad);

        // follow relative to camera motion
        robot_cam.Follow(cam_frame.Inverse());

//        pangolin::glDrawAxis(0.5);

        prog.Bind();
        prog.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
        I.SetIdentity();
        prog.SetUniform("M", I);
        prog.Unbind();
        texture_shader.Bind();
        texture_shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
        texture_shader.Unbind();

        env->render(prog);

        label_shader.Bind();
        label_shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
        I.SetIdentity();
        label_shader.SetUniform("M", I);
        label_shader.SetUniform("label_colour", pangolin::Colour::Blue());
        label_shader.Unbind();

        robot.addSkip("upperNeckPitchLink");
        //robot.render(texture_shader);
        robot.render(label_shader);
        robot.resetSkip();

        robot_cam.Unfollow();

        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
        prog.Unbind();
        obj->render(prog);

        // off-screen rendering
        if(store_img && export_colour) {
            // new view at higher resolution
            glViewport(0,0,w,h);

            // activate framebuffer
            fbo_buffer.Bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // render START
            // render into frame buffer
//            glColor3f(1.0,1.0,1.0);
//            pangolin::glDrawColouredCube();

            robot_cam.Follow(cam_frame.Inverse());

            prog.Bind();
            prog.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
            I.SetIdentity();
            prog.SetUniform("M", I);
            prog.Unbind();
            texture_shader.Bind();
            texture_shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
            texture_shader.Unbind();

            if(save_background) {
                env->render(prog);
            }

            if(save_robot) {
                robot.addSkip("upperNeckPitchLink");
                //robot.render(texture_shader);
                robot.render(label_shader, true);
                robot.resetSkip();
            }

            robot_cam.Unfollow();

            if(save_object) {
                prog.Bind();
                prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
                prog.Unbind();
                obj->render(prog);
            }
            // render END

            glFlush();

            // write image
            pangolin::Image<unsigned char> buffer;
            pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGB24");
            buffer.Alloc(w, h, w * fmt.bpp/8 );
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0,0,w,h, GL_RGB, GL_UNSIGNED_BYTE, buffer.ptr );
            pangolin::SaveImage(buffer, fmt, export_dirs.at("colour") / ("colour_"+std::to_string(iimg)+".png"), false);
            buffer.Dealloc();

            // deactivate frame buffer
            fbo_buffer.Unbind();
        }

        /// label
        label_view.Activate(robot_cam);

        // coordinates in camera frame: look from origin in Z-direction
        robot_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,0,0,0,3,pangolin::AxisY));

        // follow relative to camera motion
        robot_cam.Follow(cam_frame.Inverse());

//        pangolin::glDrawAxis(0.5);

        label_shader.Bind();
        label_shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
        I.SetIdentity();
        label_shader.SetUniform("M", I);
        label_shader.SetUniform("label_colour", pangolin::Colour::Blue());
        label_shader.Unbind();

        env->render(label_shader);

        label_shader.Bind();
        label_shader.SetUniform("label_colour", pangolin::Colour::Red());
        label_shader.Unbind();
        robot.addSkip("upperNeckPitchLink");
        robot.render(label_shader, false);
        robot.resetSkip();

        robot_cam.Unfollow();

        label_shader.Bind();
        label_shader.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
        label_shader.SetUniform("label_colour", pangolin::Colour::Green());
        label_shader.Unbind();
        obj->render(label_shader);

        // off-screen rendering
        if(store_img && export_part_masks) {
            glViewport(0,0,w,h);
            fbo_buffer.Bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            std::map<std::string, pangolin::Image<uint8_t>> masks;
            label_shader.Bind();
            label_shader.SetUniform("label_colour", pangolin::Colour::Red());
            label_shader.Unbind();
            robot.addSkip("upperNeckPitchLink");
            robot.render(label_shader, false, &masks);
            robot.resetSkip();

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            label_shader.Bind();
            label_shader.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
            label_shader.SetUniform("label_colour", pangolin::Colour::Green());
            label_shader.Unbind();
            obj->render(label_shader);

            pangolin::Image<uint8_t> buffer;
            buffer.Alloc(w, h, w);
            glReadBuffer(GL_BACK);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0,0,w,h, GL_ALPHA, GL_UNSIGNED_BYTE, buffer.ptr );
            masks["object"] = buffer;

            fbo_buffer.Unbind();

            for(auto& m : masks) {
                pangolin::PixelFormat fmt_alpha = pangolin::PixelFormatFromString("GRAY8");
                pangolin::SaveImage(m.second, fmt_alpha, export_dirs.at("masks") / ("label_"+std::to_string(iimg)+"_"+m.first+".png"), false);
                // release memory
                m.second.Dealloc();
            }
        }

        // off-screen rendering
        if(store_img && (export_depth_viz || export_depth || export_label)) {
            // new view at higher resolution
            glViewport(0,0,w,h);

            // activate framebuffer
            fbo_buffer.Bind();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // render START

            robot_cam.Follow(cam_frame.Inverse());

            if(save_background) {
                label_shader.Bind();
                label_shader.SetUniform("MVP", robot_cam.GetProjectionModelViewMatrix());
                I.SetIdentity();
                label_shader.SetUniform("M", I);
                label_shader.SetUniform("label_colour", pangolin::Colour::Blue());
                label_shader.Unbind();
                env->render(label_shader);
            }

            if(save_robot) {
                label_shader.Bind();
                label_shader.SetUniform("label_colour", pangolin::Colour::Red());
                label_shader.Unbind();
                robot.addSkip("upperNeckPitchLink");
                robot.render(label_shader, false);
                robot.resetSkip();
            }

            robot_cam.Unfollow();

            if(save_object) {
                label_shader.Bind();
                label_shader.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix(grasp_frame)*T_po);
                label_shader.SetUniform("label_colour", pangolin::Colour::Green());
                label_shader.Unbind();
                obj->render(label_shader);
            }
            // render END

            glFlush();

            // write label image
            if(export_label) {
                pangolin::Image<unsigned char> buffer;
                pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGB24");
                buffer.Alloc(w, h, w * fmt.bpp/8 );
                glReadBuffer(GL_BACK);
                glPixelStorei(GL_PACK_ALIGNMENT, 1);
                glReadPixels(0,0,w,h, GL_RGB, GL_UNSIGNED_BYTE, buffer.ptr );
                pangolin::SaveImage(buffer, fmt, export_dirs.at("label") / ("label_"+std::to_string(iimg)+".png"), false);
                buffer.Dealloc();
            }

            if(export_depth_viz || export_depth) {
                // depth data
                glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, depth_data.data());
                double tempX, tempY, tempZ;
                // row-major reading of depth from camera
                for(double y(0); y<h; y++) {
                    for(double x(0); x<w; x++) {
                        uint index = uint(y)*w+uint(x);
                        robot_view.GetObjectCoordinates(robot_cam, x, y, depth_data[index], tempX, tempY, tempZ);
                        depth_data_mm[index]  = (tempZ>depth_cutoff) ? depth_max_export_mm : tempZ*1000; // mm
                        depth_data_vis[index] = (tempZ>depth_cutoff) ? 0 : tempZ /2 * 255; // 2m = 255
                    }
                }

                if(export_depth_viz) {
                    pangolin::Image<uint8_t> depth_img_vis;
                    pangolin::PixelFormat depth_fmt = pangolin::PixelFormatFromString("GRAY8");
                    depth_img_vis.Alloc(w, h, w * depth_fmt.bpp/8 );
                    memcpy(depth_img_vis.ptr, depth_data_vis.data(), sizeof(uint8_t)*w*h);
                    pangolin::SaveImage(depth_img_vis, depth_fmt, export_dirs.at("depth_viz")  / ("depth_viz_"+std::to_string(iimg)+".png"), false);
                    depth_img_vis.Dealloc();
                }

                if(export_depth) {
                    // store depth values in mm as 16bit image
                    cv::Mat depth_img(h, w, CV_16UC1, depth_data_mm.data());
                    // flip around x-axis, origin from top left to bottom left
                    cv::flip(depth_img, depth_img, 0);
                    cv::imwrite(export_dirs.at("depth") / ("depth_"+std::to_string(iimg)+".png"), depth_img);
                }
            }

            // deactivate frame buffer
            fbo_buffer.Unbind();

            // export link pose in camera frame
            for(const std::string& link_name : export_link_pose_names) {
                KDL::Frame link_pose = robot.getLinkPoseInCameraFrame(link_name);

                if(std::string(cpose_string).size()>0) {
                    // expressed in base frame
                    // if variable 'camera_pose' is given, we assume that it is
                    // not part of the kinematic chain and 'link_pose' is the
                    // pose of the link frame in the base frame

                    KDL::Frame camera_pose;
                    camera_pose.p = KDL::Vector(cam_pose_iso3.translation().x(), cam_pose_iso3.translation().y(), cam_pose_iso3.translation().z());
                    camera_pose.M = KDL::Rotation::Quaternion(Eigen::Quaternionf(cam_pose_iso3.rotation()).x(),
                                                              Eigen::Quaternionf(cam_pose_iso3.rotation()).y(),
                                                              Eigen::Quaternionf(cam_pose_iso3.rotation()).z(),
                                                              Eigen::Quaternionf(cam_pose_iso3.rotation()).w()); //xyzw

                    const KDL::Frame Tcp = camera_pose.Inverse() * link_pose;

                    // express link_pose in camera frame
                    link_pose = Tcp;
                }

                // 6D frame pose in camera frame
                double qx, qy, qz, qw;
                link_pose.M.GetQuaternion(qx, qy, qz, qw);
                (*pose_export_files[link_name]) << link_pose.p.x() << " " << link_pose.p.y() << " " << link_pose.p.z() << " ";
                (*pose_export_files[link_name]) << qw << " " << qx << " " << qy << " " << qz;
                (*pose_export_files[link_name]) << std::endl;

                // 2D frame position in image plane
                const double wx = centre_x + link_pose.p.x()/link_pose.p.z() * f_u;
                const double wy = centre_y + link_pose.p.y()/link_pose.p.z() * f_v;
                (*joint_pos_export_files[link_name]) << wx << " " << wy << std::endl;
            } // frame pose export
        }

        // draw
        pangolin::FinishFrame();
    } // pangolin loop

    // close pose export files
    for(const std::pair<std::string, std::shared_ptr<std::ofstream>> lf : pose_export_files) {
        lf.second->close();
    }

    joint_time.close();

    delete lcm;
    delete log;

    return 0;
}
