#include "StlIO.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/STL.h>

#include <array>
#include <vector>

namespace SPMeshHealer {

bool readSTL(const std::string& path, RawMeshData& out) {
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    std::vector<std::array<double, 3>> points;
    std::vector<std::vector<std::size_t>> facets;
    if (!CGAL::IO::read_STL(path, points, facets)) return false;

    out.verts.resize(points.size() * 3);
    for (size_t i = 0; i < points.size(); ++i) {
        out.verts[i * 3 + 0] = static_cast<float>(points[i][0]);
        out.verts[i * 3 + 1] = static_cast<float>(points[i][1]);
        out.verts[i * 3 + 2] = static_cast<float>(points[i][2]);
    }

    out.faces.resize(facets.size() * 4, 0xFFFFFFFFu);
    for (size_t f = 0; f < facets.size(); ++f) {
        const auto& poly = facets[f];
        for (size_t k = 0; k < poly.size() && k < 4; ++k) {
            out.faces[f * 4 + k] = static_cast<uint32_t>(poly[k]);
        }
    }
    return true;
}

bool writeSTL(const std::string& path, const RawMeshData& data) {
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point_3 = K::Point_3;

    const size_t nbVerts = data.vertCount();
    const size_t nbFaces = data.faceCount();
    std::vector<Point_3> points(nbVerts);
    for (size_t i = 0; i < nbVerts; ++i) {
        points[i] = Point_3(data.verts[i * 3 + 0], data.verts[i * 3 + 1], data.verts[i * 3 + 2]);
    }

    std::vector<std::vector<std::size_t>> facets;
    facets.reserve(nbFaces);
    for (size_t f = 0; f < nbFaces; ++f) {
        uint32_t v0 = data.faces[f * 4 + 0];
        uint32_t v1 = data.faces[f * 4 + 1];
        uint32_t v2 = data.faces[f * 4 + 2];
        uint32_t v3 = data.faces[f * 4 + 3];
        if (v3 != 0xFFFFFFFFu) {
            facets.push_back({v0, v1, v2});
            facets.push_back({v0, v2, v3});
        } else {
            facets.push_back({v0, v1, v2});
        }
    }

    return CGAL::IO::write_STL(path, points, facets, CGAL::parameters::use_binary_mode(true));
}

} // namespace SPMeshHealer
