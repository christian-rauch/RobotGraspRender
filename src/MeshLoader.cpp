#include "MeshLoader.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>
#include <cassert>

MeshLoader::MeshLoader() { }

std::vector<MeshPtr> MeshLoader::getMesh(const std::string &path) {
    Assimp::Importer importer;

    //const uint flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices;
    const uint flags = aiProcess_Triangulate;

    const aiScene* scene = importer.ReadFile(path, flags);

    if(!scene) {
        std::cerr<<"Import error: "<<importer.GetErrorString()<<std::endl;
    }

    const uint nMeshes = scene->mNumMeshes;

    std::cout<<"meshes: "<<nMeshes<<std::endl;

    std::vector<MeshPtr> meshes;

    for(uint iMesh(0); iMesh<nMeshes; iMesh++) {
        const aiMesh *aimesh = scene->mMeshes[iMesh];

        std::cout<<"name: "<<aimesh->mName.C_Str()<<std::endl;

        std::cout<<"colour?: "<<aimesh->GetNumColorChannels()<<std::endl;
        std::cout<<"UV?: "<<aimesh->GetNumUVChannels()<<std::endl;

        const uint nVerts = aimesh->mNumVertices;
        const uint nFaces = aimesh->mNumFaces;

        Mesh mesh(nVerts, nFaces);

        std::cout<<"verts: "<<nVerts<<std::endl;
        std::cout<<"faces: "<<nFaces<<std::endl;

        for(uint i = 0; i<nVerts; i++) {
            // vertices
            mesh.vertices.row(i) << aimesh->mVertices[i].x, aimesh->mVertices[i].y, aimesh->mVertices[i].z;

            // normals
            if(aimesh->HasNormals())
                mesh.normals.row(i) << aimesh->mNormals[i].x, aimesh->mNormals[i].y, aimesh->mNormals[i].z;

            // texture coordiantes
            if(aimesh->mTextureCoords[0]!=NULL)
                mesh.texture.row(i) << aimesh->mTextureCoords[0][i].x, aimesh->mTextureCoords[0][i].y;

            // colour, r, g, b
            if(aimesh->mColors[0]!=NULL)
                mesh.colour.row(i) << aimesh->mColors[0][i].r, aimesh->mColors[0][i].g, aimesh->mColors[0][i].b;

            // TODO: material
        }

        for(uint i = 0; i<nFaces; i++) {
            const aiFace face = aimesh->mFaces[i];

            // only accept triangulated meshes
            assert(face.mNumIndices==3);

            mesh.faces.row(i) << face.mIndices[0], face.mIndices[1], face.mIndices[2];
        }

        meshes.push_back(std::make_shared<Mesh>(mesh));
    } // for all nMeshes

    return meshes;
}
