#include "Mesh.hpp"

#include <pangolin/gl/gl.hpp>
#include <pangolin/gl/glvbo.h>

#define VOXELIZER_IMPLEMENTATION
#include "voxelizer.h"

Mesh::Mesh() {}

void Mesh::renderSetup() {
    //// create buffers
    // vertices
    vertexbuffer.Reinitialise(pangolin::GlArrayBuffer, vertices.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
    vertexbuffer.Upload(vertices.data(), sizeof(float)*vertices.size()*3);

    // indices
    elementbuffer.Reinitialise(pangolin::GlElementArrayBuffer, faces.size(), GL_UNSIGNED_INT, 3, GL_STATIC_DRAW);
    elementbuffer.Upload(faces.data(), sizeof(uint)*faces.size()*3);

    // colour
    colourbuffer.Reinitialise(pangolin::GlArrayBuffer, colour.size(), GL_FLOAT, 3, GL_STATIC_DRAW);
    colourbuffer.Upload(colour.data(), sizeof(float)*colour.size()*3);

    // UV (texture coordinates)
    uvbuffer.Reinitialise(pangolin::GlArrayBuffer, uv.size(), GL_FLOAT, 2, GL_STATIC_DRAW);
    uvbuffer.Upload(uv.data(), sizeof(float)*uv.size()*2);

    // load texture
    if(texture.SizeBytes()>0) {
        gl_texture.Load(texture);
        gl_texture.SetLinear();
    }
}


void Mesh::render(pangolin::GlSlProgram &shader) {
    shader.Bind();

    if(texture.SizeBytes()>0) {
        gl_texture.Bind();

    vertexbuffer.Bind();
    glVertexPointer(vertexbuffer.count_per_element, vertexbuffer.datatype, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);

    elementbuffer.Bind();

    if(texture.SizeBytes()>0) {

        uvbuffer.Bind();
        //glEnableVertexAttribArray(uvbuffer.bo);
        glEnableVertexAttribArray(4);
        //glVertexAttribPointer( uvbuffer.bo, 2, GL_FLOAT, GL_FALSE, 0, (void*)0 );
        glVertexAttribPointer( 4, 2, GL_FLOAT, GL_FALSE, 0, (void*)0 );

        glDrawElements(GL_TRIANGLES, elementbuffer.num_elements*elementbuffer.count_per_element, elementbuffer.datatype, 0);

        glDisableVertexAttribArray(uvbuffer.bo);
        uvbuffer.Unbind();

        gl_texture.Unbind();
    }

    elementbuffer.Unbind();

    glDisableClientState(GL_VERTEX_ARRAY);
    vertexbuffer.Unbind();

    } // texture
    else {
        if(hasColour()) {
            colourbuffer.Bind();
            glColorPointer(colourbuffer.count_per_element, colourbuffer.datatype, 0, 0);
            glEnableClientState(GL_COLOR_ARRAY);
        }

        vertexbuffer.Bind();
        glVertexPointer(vertexbuffer.count_per_element, vertexbuffer.datatype, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);

        elementbuffer.Bind();

        glDrawElements(GL_TRIANGLES,elementbuffer.num_elements*elementbuffer.count_per_element, elementbuffer.datatype, 0);

        elementbuffer.Unbind();

        glDisableClientState(GL_VERTEX_ARRAY);
        vertexbuffer.Unbind();

        if(hasColour()) {
            glDisableClientState(GL_COLOR_ARRAY);
            colourbuffer.Unbind();
        }
    }

    shader.Unbind();
}

void Mesh::voxelize(const float resolution, const float precision) {
    // remove texture for voxelized meshes
    uv.clear();
    texture.Reinitialise(0,0,texture.fmt);

    // define mesh
    vx_mesh_t *mesh = vx_mesh_alloc(vertices.size(), faces.size()*3);

    mesh->vertices = reinterpret_cast<vx_vertex_t*>(vertices.data());
    mesh->nvertices = vertices.size();

    mesh->normals = reinterpret_cast<vx_vec3_t*>(normals.data());
    mesh->nnormals = normals.size();

    // indexes of faces as list
    mesh->indices = reinterpret_cast<unsigned int*>(faces.data());
    mesh->nindices = faces.size()*3;

    // voxelize
    vx_mesh_t *voxel_mesh = vx_voxelize(mesh, resolution, resolution, resolution, precision);

    // replace mesh by voxelized version
    vertices.resize(voxel_mesh->nvertices);
    memcpy(vertices.data(), voxel_mesh->vertices, sizeof(vx_vertex_t)*voxel_mesh->nvertices);

    normals.resize(voxel_mesh->nnormals);
    memcpy(normals.data(), voxel_mesh->normals, sizeof(vx_vec3_t)*voxel_mesh->nnormals);

    faces.resize(voxel_mesh->nindices/3);
    memcpy(faces.data(), voxel_mesh->indices, sizeof(unsigned int)*voxel_mesh->nindices);

    // do not free 'mesh' as it shares data, only delete struct
    vx_mesh_free(voxel_mesh);
    delete mesh;
}
