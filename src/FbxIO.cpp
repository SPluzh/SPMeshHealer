#include "FbxIO.h"
#include "ufbx/ufbx.h"

#include <fstream>
#include <vector>

namespace SPMeshHealer {

bool readFBX(const std::string& path, RawMeshData& out) {
    ufbx_load_opts opts{};
    opts.target_axes = ufbx_axes_right_handed_y_up;
    opts.generate_missing_normals = false;
    ufbx_error err{};
    ufbx_scene* scene = ufbx_load_file(path.c_str(), &opts, &err);
    if (!scene) return false;

    uint32_t vertOffset = 0;
    std::vector<uint32_t> triIndices;

    for (size_t ni = 0; ni < scene->nodes.count; ++ni) {
        ufbx_node* node = scene->nodes.data[ni];
        if (!node->mesh) continue;
        ufbx_mesh* mesh = node->mesh;
        if (mesh->num_vertices == 0 || mesh->num_faces == 0) continue;

        if (out.meshName == "Mesh" && node->name.length > 0) {
            out.meshName.assign(node->name.data, node->name.length);
        }

        const ufbx_matrix xform = node->geometry_to_world;
        const size_t nv = mesh->num_vertices;
        out.verts.reserve(out.verts.size() + nv * 3);

        for (size_t vi = 0; vi < nv; ++vi) {
            ufbx_vec3 p = ufbx_transform_position(&xform, mesh->vertices.data[vi]);
            out.verts.push_back(static_cast<float>(p.x));
            out.verts.push_back(static_cast<float>(p.y));
            out.verts.push_back(static_cast<float>(p.z));
        }

        triIndices.resize(mesh->max_face_triangles * 3);
        for (size_t fi = 0; fi < mesh->num_faces; ++fi) {
            ufbx_face face = mesh->faces.data[fi];
            if (face.num_indices == 3) {
                uint32_t i0 = vertOffset + mesh->vertex_indices.data[face.index_begin + 0];
                uint32_t i1 = vertOffset + mesh->vertex_indices.data[face.index_begin + 1];
                uint32_t i2 = vertOffset + mesh->vertex_indices.data[face.index_begin + 2];
                out.faces.push_back(i0);
                out.faces.push_back(i1);
                out.faces.push_back(i2);
                out.faces.push_back(0xFFFFFFFFu);
            } else if (face.num_indices == 4) {
                uint32_t i0 = vertOffset + mesh->vertex_indices.data[face.index_begin + 0];
                uint32_t i1 = vertOffset + mesh->vertex_indices.data[face.index_begin + 1];
                uint32_t i2 = vertOffset + mesh->vertex_indices.data[face.index_begin + 2];
                uint32_t i3 = vertOffset + mesh->vertex_indices.data[face.index_begin + 3];
                out.faces.push_back(i0);
                out.faces.push_back(i1);
                out.faces.push_back(i2);
                out.faces.push_back(i3);
            } else if (face.num_indices > 4) {
                uint32_t nTris = ufbx_triangulate_face(triIndices.data(), triIndices.size(), mesh, face);
                for (uint32_t ti = 0; ti < nTris; ++ti) {
                    out.faces.push_back(vertOffset + mesh->vertex_indices.data[triIndices[ti * 3 + 0]]);
                    out.faces.push_back(vertOffset + mesh->vertex_indices.data[triIndices[ti * 3 + 1]]);
                    out.faces.push_back(vertOffset + mesh->vertex_indices.data[triIndices[ti * 3 + 2]]);
                    out.faces.push_back(0xFFFFFFFFu);
                }
            }
        }

        vertOffset += static_cast<uint32_t>(nv);
    }

    ufbx_free_scene(scene);
    return out.vertCount() > 0 && out.faceCount() > 0;
}

bool writeFBX(const std::string& path, const RawMeshData& data) {
    const size_t nbVerts = data.vertCount();
    const size_t nbFaces = data.faceCount();
    if (nbVerts == 0 || nbFaces == 0) return false;

    std::ofstream out(path);
    if (!out.is_open()) return false;

    size_t polyIndexCount = 0;
    for (size_t f = 0; f < nbFaces; ++f) {
        polyIndexCount += (data.faces[f * 4 + 3] != 0xFFFFFFFFu) ? 4 : 3;
    }

    out << "; FBX 7.4.0 project file (SPMeshHealer)\n"
        << "FBXHeaderExtension:  {\n"
        << "\tFBXHeaderVersion: 1003\n"
        << "\tFBXVersion: 7400\n"
        << "}\n"
        << "Definitions:  {\n"
        << "\tVersion: 100\n"
        << "\tObjectType: \"Geometry\" {\n\t\tCount: 1\n\t}\n"
        << "\tObjectType: \"Model\" {\n\t\tCount: 1\n\t}\n"
        << "}\n"
        << "Objects:  {\n"
        << "\tGeometry: 1000, \"Geometry::" << data.meshName << "\", \"Mesh\" {\n"
        << "\t\tVertices: *" << (nbVerts * 3) << " {\n\t\t\ta: ";

    for (size_t i = 0; i < nbVerts * 3; ++i) {
        if (i > 0) out << ',';
        if (i > 0 && (i % 12) == 0) out << "\n\t\t\t";
        out << data.verts[i];
    }
    out << "\n\t\t}\n"
        << "\t\tPolygonVertexIndex: *" << polyIndexCount << " {\n\t\t\ta: ";

    size_t written = 0;
    for (size_t f = 0; f < nbFaces; ++f) {
        uint32_t v0 = data.faces[f * 4 + 0];
        uint32_t v1 = data.faces[f * 4 + 1];
        uint32_t v2 = data.faces[f * 4 + 2];
        uint32_t v3 = data.faces[f * 4 + 3];
        if (written > 0) out << ',';
        if (written > 0 && (written % 12) == 0) out << "\n\t\t\t";
        if (v3 != 0xFFFFFFFFu) {
            int last = -static_cast<int>(v3) - 1;
            out << v0 << ',' << v1 << ',' << v2 << ',' << last;
            written += 4;
        } else {
            int last = -static_cast<int>(v2) - 1;
            out << v0 << ',' << v1 << ',' << last;
            written += 3;
        }
    }
    out << "\n\t\t}\n"
        << "\t\tGeometryVersion: 124\n"
        << "\t}\n"
        << "\tModel: 2000, \"Model::" << data.meshName << "\", \"Mesh\" {\n"
        << "\t\tVersion: 232\n"
        << "\t\tShading: T\n"
        << "\t\tCulling: \"CullingOff\"\n"
        << "\t}\n"
        << "}\n"
        << "Connections:  {\n"
        << "\tC: \"OO\",2000,0\n"
        << "\tC: \"OO\",1000,2000\n"
        << "}\n";

    return out.good();
}

} // namespace SPMeshHealer
