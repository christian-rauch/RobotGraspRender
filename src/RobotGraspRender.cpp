#include <pangolin/pangolin.h>

#include <MeshLoader.hpp>

#include <RobotModel.hpp>

int main(int argc, char *argv[]) {

    pangolin::ParseVarsFile(argv[1]);

    pangolin::Var<std::string> env_path("environment_mesh");
    pangolin::Var<std::string> obj_path("object_mesh");
    pangolin::Var<std::string> robot_model_path("robot_model");

    std::cout<<"robot: "<<robot_model_path<<std::endl;
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
    }

    // meshes
    MeshPtr env;
    if(env_path->empty()) {
        env = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        env = MeshLoader::getMesh(env_path);
    }

    MeshPtr obj;
    if(obj_path->empty()) {
        obj = std::unique_ptr<Mesh>(new Mesh());
    }
    else {
        obj = MeshLoader::getMesh(obj_path);
    }

//    std::cout<<"verts: "<<obj->vertices.rows()<<std::endl;
//    std::cout<<"faces: "<<obj->faces.rows()<<std::endl;

    std::cout<<"verts: "<<obj->vertices.size()<<std::endl;
    std::cout<<"faces: "<<obj->faces.size()<<std::endl;

//    pangolin::GlBuffer vertexbuffer(pangolin::GlArrayBuffer, obj->vertices.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
//    vertexbuffer.Upload(obj->vertices.data(), sizeof(float)*obj->vertices.size()*3);

//    pangolin::GlBuffer elementbuffer(pangolin::GlElementArrayBuffer, obj->faces.size(), GL_UNSIGNED_INT, 3, GL_STATIC_DRAW);
//    elementbuffer.Upload(obj->faces.data(), sizeof(uint)*obj->faces.size()*3);

//    pangolin::GlBuffer colourbuffer(pangolin::GlArrayBuffer, obj->colour.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
//    colourbuffer.Upload(obj->colour.data(), sizeof(float)*obj->colour.size()*3);

//    pangolin::GlBuffer uvbuffer(pangolin::GlArrayBuffer, obj->uv.size(), GL_FLOAT, 2, GL_STATIC_DRAW);
//    uvbuffer.Upload(obj->uv.data(), sizeof(float)*obj->uv.size()*2);

//    pangolin::GlTexture texture;
//    if(obj->texture.SizeBytes()>0) {
//        texture.Load(obj->texture);
//        texture.SetLinear();
//    }

//    GLuint vertexbuffer;
//    glGenBuffers(1, &vertexbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*obj->vertices.size()*3, obj->vertices.data(), GL_STATIC_DRAW);
//    GLuint elementbuffer;
//    glGenBuffers(1, &elementbuffer);
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*obj->faces.size()*3, obj->faces.data(), GL_STATIC_DRAW);
//    GLuint colorbuffer;
//    glGenBuffers(1, &colorbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*obj->colour.size()*3, obj->colour.data(), GL_STATIC_DRAW);

//    GLuint uvbuffer;
//    glGenBuffers(1, &uvbuffer);
//    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    env->renderSetup();
    obj->renderSetup();

//    robot.link_meshes["hokuyo_link"]->renderSetup();
    robot.renderSetup();



    ////////////////////////////////////////////////////////////////////////////
    /// Draw
    while(!pangolin::ShouldQuit()) {

        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

//        pangolin::glDrawColouredCube();
        pangolin::glDrawAxis(1);

//        prog.Bind();
        prog.Bind();
        prog.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        prog.Unbind();
        texture_shader.Bind();
        texture_shader.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
        texture_shader.Unbind();
//        texture_shader.Bind();
        // update the vertex positions based on our camera view
//        prog.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
//        texture_shader.SetUniform("MVP", s_cam.GetProjectionModelViewMatrix());
//        texture_shader.GetUniformHandle("UV");
//        GLuint TextureID = texture_shader.GetUniformHandle("myTextureSampler");
//        glActiveTexture(GL_TEXTURE0);
//        glUniform1i(TextureID, 0);
//        texture_shader.SetUniform("myTextureSampler", texture.tid);

         //draw vertices
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

//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

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

////        glDrawArrays(GL_TRIANGLES, 0, obj->faces.size());

//        glDrawElements(
//            GL_TRIANGLES,      // mode
//            obj->faces.size()*3, // count
//            GL_UNSIGNED_INT,   // type
//            (void*)0           // element array buffer offset
//        );

//        glDisableVertexAttribArray(0);
//        glDisableVertexAttribArray(1);

//        texture.Bind();


////        pangolin::RenderVbo(vertexbuffer, GL_TRIANGLES);
////        pangolin::RenderVbo(vertexbuffer, GL_TRIANGLE_STRIP);
////        pangolin::RenderVbo(vertexbuffer, GL_POINTS);
////        glEnableVertexAttribArray(0);
//        vertexbuffer.Bind();
//        glVertexPointer(vertexbuffer.count_per_element, vertexbuffer.datatype, 0, 0);
//        glEnableClientState(GL_VERTEX_ARRAY);

//        elementbuffer.Bind();

//        // draw texture
//        uvbuffer.Bind();
//        glEnableVertexAttribArray(uvbuffer.bo);
//        glVertexAttribPointer(
//            uvbuffer.bo,                                // attribute. No particular reason for 1, but must match the layout in the shader.
//            2,                                // size : U+V => 2
//            GL_FLOAT,                         // type
//            GL_FALSE,                         // normalized?
//            0,                                // stride
//            (void*)0                          // array buffer offset
//        );

//        glDrawElements(GL_TRIANGLES, elementbuffer.num_elements*elementbuffer.count_per_element, elementbuffer.datatype, 0);

//        glDisableVertexAttribArray(uvbuffer.bo);
//        uvbuffer.Unbind();

//        elementbuffer.Unbind();

//        glDisableClientState(GL_VERTEX_ARRAY);
//        vertexbuffer.Unbind();


////        pangolin::RenderVboIbo(vertexbuffer, elementbuffer, true);
////        pangolin::RenderVboIboCbo(vertexbuffer, elementbuffer, colourbuffer, true, obj->hasColour());

////        pangolin::RenderVboCbo(vertexbuffer, colourbuffer, true);

//        texture.Unbind();

//        //prog.Unbind();

        //obj->render(texture_shader);
        env->render(prog);
        obj->render(prog);

//        robot.link_meshes["hokuyo_link"]->render(texture_shader);
        robot.render(texture_shader);

//        texture_shader.Unbind();

        // draw
        pangolin::FinishFrame();
    }

    return 0;
}
