#include <pangolin/pangolin.h>

#include <MeshLoader.hpp>

int main(int argc, char *argv[]) {

    pangolin::ParseVarsFile(argv[1]);

    pangolin::Var<std::string> env_path("environment_mesh");
    pangolin::Var<std::string> obj_path("object_mesh");

    std::cout<<"environment: "<<env_path<<std::endl;
    std::cout<<"object: "<<obj_path<<std::endl;

    ////////////////////////////////////////////////////////////////////////////
    /// Window Setup
    pangolin::CreateWindowAndBind("RobotGraspRender",640,480);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
      pangolin::ModelViewLookAt(-0,0.5,-3, 0,0,0, pangolin::AxisY)
    );

    pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));

    // VAO
//    GLuint VertexArrayID;
//    glGenVertexArrays(1, &VertexArrayID);
//    glBindVertexArray(VertexArrayID);

    // load and compile shaders
    pangolin::GlSlProgram prog;
    prog.AddShaderFromFile(pangolin::GlSlVertexShader, "SimpleVertexShader.vert");
    prog.AddShaderFromFile(pangolin::GlSlFragmentShader, "SimpleFragmentShader.frag");
    prog.Link();

    ////////////////////////////////////////////////////////////////////////////
    /// Load Resources

    std::vector<MeshPtr> env;
    if(env_path->empty()) {
        env.push_back(std::make_shared<Mesh>(0,0));
    }
    else {
        env = MeshLoader::getMesh(env_path);
    }

    std::vector<MeshPtr> obj;
    if(obj_path->empty()) {
        obj.push_back(std::make_shared<Mesh>(0,0));
    }
    else {
        obj = MeshLoader::getMesh(obj_path);
    }

    std::cout<<"verts: "<<obj[0]->vertices.rows()<<std::endl;
    std::cout<<"faces: "<<obj[0]->faces.rows()<<std::endl;

    pangolin::GlBuffer vertexbuffer(pangolin::GlArrayBuffer, obj[0]->vertices.rows(), GL_FLOAT, 3, GL_STATIC_DRAW);
    vertexbuffer.Upload(obj[0]->vertices.data(), sizeof(float)*obj[0]->vertices.size());

//    pangolin::GlBuffer elementbuffer(pangolin::GlElementArrayBuffer, obj[0]->faces.rows(), GL_UNSIGNED_INT, 3, GL_STATIC_DRAW);
//    elementbuffer.Upload(obj[0]->faces.data(), sizeof(GLuint)*obj[0]->faces.size());

    pangolin::GlBuffer colourbuffer(pangolin::GlArrayBuffer, obj[0]->colour.rows(), GL_FLOAT, 3, GL_STATIC_DRAW);
    colourbuffer.Upload(obj[0]->colour.data(), sizeof(float)*obj[0]->colour.size());

    // each vertex
//    GLuint vertexbuffer;
//    glGenBuffers(1, &vertexbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*obj[0]->vertices.size(), obj[0]->getVertices(), GL_STATIC_DRAW);

//    // draw faces
//    GLuint elementbuffer;
//    glGenBuffers(1, &elementbuffer);
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*obj[0]->faces.size(), obj[0]->getFaces(), GL_STATIC_DRAW);

//    // colour
//    GLuint colorbuffer;
//    glGenBuffers(1, &colorbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*obj[0]->colour.size(), obj[0]->colour.data(), GL_STATIC_DRAW);

    ////////////////////////////////////////////////////////////////////////////
    /// Draw
    while(!pangolin::ShouldQuit()) {

        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



        d_cam.Activate(s_cam);

//        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1);

        prog.Bind();
        // update the vertex positions based on our camera view
        prog.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
//        prog.SetUniform("vertexColor", obj[0]->colour.data());

        // draw vertices
//        glEnableVertexAttribArray(0);

//        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//        glVertexAttribPointer(
//           0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
//           3,                  // size
//           GL_FLOAT,           // type
//           GL_FALSE,           // normalized?
//           0,                  // stride
//           (void*)0            // array buffer offset
//        );

        // draw faces
//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

        // colour
//        glEnableVertexAttribArray(1);
//        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
//        glVertexAttribPointer(
//            1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
//            3,                                // size
//            GL_FLOAT,                         // type
//            GL_FALSE,                         // normalized?
//            0,                                // stride
//            (void*)0                          // array buffer offset
//        );

//        glDrawArrays(GL_TRIANGLES, 0, obj[0]->faces.size());

//        glDrawElements(
//            GL_TRIANGLES,      // mode
//            obj[0]->faces.size(), // count
//            GL_UNSIGNED_INT,   // type
//            (void*)0           // element array buffer offset
//        );

//        glDisableVertexAttribArray(0);
//        glDisableVertexAttribArray(1);



        //pangolin::RenderVbo(vertexbuffer, GL_TRIANGLES);
        //pangolin::RenderVbo(vertexbuffer, GL_POINTS);
        //pangolin::RenderVboIbo(vertexbuffer, elementbuffer, true);
        //pangolin::RenderVboIboCbo(vertexbuffer, elementbuffer, colourbuffer, true, true);

        pangolin::RenderVboCbo(vertexbuffer, colourbuffer, true, GL_TRIANGLES);

        prog.Unbind();

        // draw
        pangolin::FinishFrame();
    }

    return 0;
}
