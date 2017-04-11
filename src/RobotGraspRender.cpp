#include <pangolin/pangolin.h>

#include <MeshLoader.hpp>

#include <RobotModel.hpp>

#include <lcm/lcm-cpp.hpp>
#include <robot_state_t.hpp>

#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <ios>

#include <dirent.h>
#include <sys/stat.h>

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

    bool open(const std::string &path) {
        csv_file.open(path);

        if(!csv_file.good()) {
            std::cerr<<"error openning joints file: "<<path<<std::endl;
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
                jnames.push_back(joint_name);
            }
        }
    }

    // fetch next line of joint values
    std::map<std::string, float> getNext() {
        if(jnames.size()==0)
            throw std::runtime_error("joint names not defined");

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
        return joints;
    }

private:
    std::ifstream csv_file;

    std::vector<std::string> jnames;
};

int main(int /*argc*/, char *argv[]) {
    ////////////////////////////////////////////////////////////////////////////
    /// Configuration file
    pangolin::ParseVarsFile(argv[1]);

    pangolin::Var<std::string> lcm_channel("lcm_channel");
    pangolin::Var<std::string> env_path("environment_mesh");
    pangolin::Var<std::string> obj_path("object_mesh");
    pangolin::Var<std::string> robot_model_path("robot_model");
    pangolin::Var<std::string> data_store_path("dest_dir");
    pangolin::Var<uint> nframes("save_nframes");
    pangolin::Var<std::string> logfile("log_file");
    pangolin::Var<std::string> joint_conf_path("joint_config_path");

    pangolin::Var<std::string> export_link_poses("export_link_poses");

    // camera parameters
    pangolin::Var<std::string> camera_frame("camera_frame");
    pangolin::Var<uint> cam_width("width");
    pangolin::Var<uint> cam_height("height");
    pangolin::Var<uint> centre_x("centre_x");
    pangolin::Var<uint> centre_y("centre_y");
    pangolin::Var<double> f_u("f_u");
    pangolin::Var<double> f_v("f_v");
    pangolin::Var<double> rotate_z_rad("rotate_z_rad");

    // export flags
    pangolin::Var<bool> save_background("save_background");
    pangolin::Var<bool> save_object("save_object");
    pangolin::Var<bool> save_robot("save_robot");

    pangolin::Var<bool> export_colour("export_colour");
    pangolin::Var<bool> export_depth_viz("export_depth_viz");
    pangolin::Var<bool> export_depth("export_depth");
    pangolin::Var<bool> export_label("export_label");

    std::cout<<"channel: "<<lcm_channel<<std::endl;
    std::cout<<"robot: "<<robot_model_path<<std::endl;
    std::cout<<"environment: "<<env_path<<std::endl;
    std::cout<<"object: "<<obj_path<<std::endl;
    std::cout<<"save images every "<<nframes<<" frames to: "<<data_store_path<<std::endl;

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
        std::cout<<"reading joint config from: "<<joint_conf_path<<std::endl;
        if(!csvj.open(joint_conf_path))
            std::cerr<<"could not open files"<<std::endl;
        csvj.setJointNames();
    }
    else {
        // use LCM as joint value provider
        if(!std::string(logfile).empty()) {
            // read from LCM logfile
            std::cout<<"reading from log file: "<<logfile<<std::endl;
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
    std::map<std::string, std::string> dir_names; // name, path
    dir_names["0"] = data_store_path; // name "0" to create root first
    if(export_colour)
        dir_names["colour"] = std::string(data_store_path)+"/colour/";
    if(export_depth_viz)
        dir_names["depth_viz"] = std::string(data_store_path)+"/depth_viz/";
    if(export_depth)
        dir_names["depth"] = std::string(data_store_path)+"/depth/";
    if(export_label)
        dir_names["label"] = std::string(data_store_path)+"/label/";

    for(const auto& dir : dir_names) {
        if(!opendir(std::string(dir.second).c_str()) && (errno==ENOENT)) {
            // directory does not exist
            std::cout<<"creating new directory: "<<std::string(dir.second)<<std::endl;
            // S_IRWXU: read+write by owner
            // S_IRWXG: read+write by group
            // S_IROTH: read only by others
            mkdir(std::string(dir.second).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }
    }

    // read link names for which to export pose
    std::vector<std::string> export_link_pose_names;
    {
        std::istringstream iss(export_link_poses);
        std::string link_name;
        while (std::getline(iss, link_name, ',')) {
            // remove whitespace
            link_name.erase(remove_if(link_name.begin(), link_name.end(), isspace), link_name.end());
            export_link_pose_names.push_back(link_name);
        }
    }

    // create files for pose export and write header
    std::map<std::string, std::shared_ptr<std::ofstream>> pose_export_files;
    for(const std::string& link : export_link_pose_names) {
        pose_export_files[link] = std::make_shared<std::ofstream>(std::string(data_store_path)+"/"+link+"_pose.csv");
        (*pose_export_files[link]) << "px py pz qw qx qy qz" << std::endl;
    }

    // create file for timestamps
    std::ofstream joint_time(std::string(data_store_path)+"/time.csv");

    ////////////////////////////////////////////////////////////////////////////
    /// Window Setup
    pangolin::CreateWindowAndBind("RobotGraspRender",640,480);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
      pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
    );

    // robot camera
    // multisense paramters
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
    if(!robot_model_path->empty()) {
        robot.parseURDF(robot_model_path);
        // load meshes and set unique colour
        robot.loadLinkMeshes();
        // initialising joints and pose
        robot.loadJointNames();
        robot.camera_frame_name = camera_frame;
        //robot.generateMeshColours(false);
        robot.generateMeshColours(false, true); // gray channel labels
    }

    // export link names and labels
    if(robot.link_colours.size()!=robot.link_label_id.size())
        throw std::runtime_error("link label and colour size mismatch");

    std::ofstream link_label_file(std::string(data_store_path)+"/link_label.csv");
    //link_label_file << "name id r g b"<<std::endl;
    for(auto it = robot.link_meshes.begin(); it!=robot.link_meshes.end(); it++) {
        const std::string link_name = it->first;
        // link name and id (gray value)
        link_label_file << link_name << " " << robot.link_label_id[link_name];
        // colour channels
        link_label_file << " " << robot.link_colours_rgb[link_name].r << " " <<robot.link_colours_rgb[link_name].g << " " << robot.link_colours_rgb[link_name].b;
        link_label_file << std::endl;
    }
    link_label_file.close();


    // meshes
    MeshPtr env;
    if(env_path->empty()) {
        env = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        env = MeshLoader::getMesh(env_path);
    }
//    std::cout<<"env verts: "<<env->vertices.size()<<std::endl;
//    std::cout<<"env faces: "<<env->faces.size()<<std::endl;

    MeshPtr obj;
    if(obj_path->empty()) {
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

        /// free view
        d_cam.Activate(s_cam);

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
            auto jv = csvj.getNext();
            if(!jv.empty()) {
                robot.joints = jv;
            }
            else {
                // end of CSV file
                break;
            }
        }

        // update link/mesh poses
        robot.updateFrames();

        // export hand pose in camera frame
        if(store_img) {
            for(const std::pair<std::string, std::shared_ptr<std::ofstream>> lf : pose_export_files) {
                const KDL::Frame link_pose = robot.getLinkPoseInCameraFrame(lf.first);
                double qx, qy, qz, qw;
                link_pose.M.GetQuaternion(qx, qy, qz, qw);

                (*lf.second) << link_pose.p.x() << " " << link_pose.p.y() << " " << link_pose.p.z() << " ";
                (*lf.second) << qw << " " << qx << " " << qy << " " << qz;
                (*lf.second) << std::endl;
            }
        }

        //robot.render(texture_shader);
        robot.render(label_shader);

        // get hand pose from robot and render object at this pose
        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix("l_hand_face"));
        prog.Unbind();
        obj->render(prog);

        /// robot view
        robot_view.Activate(robot_cam);

        // coordinates in camera frame: look from origin in Z-direction
        robot_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0,0,0,0,3,pangolin::AxisY));

        // follow relative to camera motion
        //const pangolin::OpenGlMatrix cam_frame = robot.T_wr*robot.getFramePose(camera_frame);
        const pangolin::OpenGlMatrix cam_frame = robot.T_wr*robot.getFramePoseMatrix(robot.camera_frame_name)*pangolin::OpenGlMatrix::RotateZ(rotate_z_rad);
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

        robot.addSkip("upperNeckPitchLink");
        robot.render(texture_shader);
        robot.resetSkip();

        robot_cam.Unfollow();

        prog.Bind();
        prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix("l_hand_face"));
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
                robot.render(texture_shader);
                robot.resetSkip();
            }

            robot_cam.Unfollow();

            if(save_object) {
                prog.Bind();
                prog.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix("l_hand_face"));
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
            pangolin::SaveImage(buffer, fmt, dir_names["colour"]+"/colour_"+std::to_string(iimg)+".png", false);
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
        robot.render(label_shader);
        robot.resetSkip();

        robot_cam.Unfollow();

        label_shader.Bind();
        label_shader.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix("l_hand_face"));
        label_shader.SetUniform("label_colour", pangolin::Colour::Green());
        label_shader.Unbind();
        obj->render(label_shader);

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
                robot.render(label_shader);
                robot.resetSkip();
            }

            robot_cam.Unfollow();

            if(save_object) {
                label_shader.Bind();
                label_shader.SetUniform("M", robot.T_wr*robot.getFramePoseMatrix("l_hand_face"));
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
                pangolin::SaveImage(buffer, fmt, dir_names["label"]+"/label_"+std::to_string(iimg)+".png", false);
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
                        depth_data_mm[index] = tempZ*1000; // mm
                        depth_data_vis[index] = tempZ /2 * 255; // 2m = 255
                    }
                }

                if(export_depth_viz) {
                    pangolin::Image<uint8_t> depth_img_vis;
                    pangolin::PixelFormat depth_fmt = pangolin::PixelFormatFromString("GRAY8");
                    depth_img_vis.Alloc(w, h, w * depth_fmt.bpp/8 );
                    memcpy(depth_img_vis.ptr, depth_data_vis.data(), sizeof(uint8_t)*w*h);
                    pangolin::SaveImage(depth_img_vis, depth_fmt, dir_names["depth_viz"]+"/depth_viz_"+std::to_string(iimg)+".png", false);
                    depth_img_vis.Dealloc();
                }

                if(export_depth) {
                    // store depth values in mm as 16bit image
                    cv::Mat depth_img(h, w, CV_16UC1, depth_data_mm.data());
                    // flip around x-axis, origin from top left to bottom left
                    cv::flip(depth_img, depth_img, 0);
                    cv::imwrite(dir_names["depth"]+"/depth_"+std::to_string(iimg)+".png", depth_img);
                }
            }

            // deactivate frame buffer
            fbo_buffer.Unbind();
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
