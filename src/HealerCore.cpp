#include "HealerCore.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

using Kernel  = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT      = Kernel::FT;
using Point_3 = Kernel::Point_3;
using Mesh    = CGAL::Surface_mesh<Point_3>;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace NP  = CGAL::parameters;

using CustomPoint = std::array<FT, 3>;
using Polygon     = std::vector<std::size_t>;

struct ArrayTraits {
    struct Equal_3 {
        bool operator()(const CustomPoint& p, const CustomPoint& q) const { return p == q; }
    };
    struct Less_xyz_3 {
        bool operator()(const CustomPoint& p, const CustomPoint& q) const {
            return std::lexicographical_compare(p.begin(), p.end(), q.begin(), q.end());
        }
    };
    Equal_3 equal_3_object() const { return Equal_3(); }
    Less_xyz_3 less_xyz_3_object() const { return Less_xyz_3(); }
};

double computeHoleBBoxDiagSq(const Mesh& mesh, Mesh::Halfedge_index seed) {
    CGAL::Bbox_3 bbox;
    Mesh::Halfedge_index h = seed;
    do {
        bbox += mesh.point(mesh.target(h)).bbox();
        h = mesh.next(h);
    } while (h != seed);
    double dx = bbox.xmax() - bbox.xmin();
    double dy = bbox.ymax() - bbox.ymin();
    double dz = bbox.zmax() - bbox.zmin();
    return dx * dx + dy * dy + dz * dz;
}

bool buildMeshDirect(
    const RawMeshData& input,
    Mesh& mesh,
    Mesh::Property_map<Mesh::Vertex_index, int>& vOrigMap,
    Mesh::Property_map<Mesh::Face_index, uint16_t>& fGroupMap,
    std::vector<Mesh::Vertex_index>& outVHandles)
{
    const size_t nbVerts = input.vertCount();
    const size_t nbFaces = input.faceCount();
    const bool hasGroups = input.faceGroups.size() == nbFaces;

    outVHandles.resize(nbVerts);
    for (size_t i = 0; i < nbVerts; ++i) {
        outVHandles[i] = mesh.add_vertex(Point_3(
            input.verts[i * 3 + 0],
            input.verts[i * 3 + 1],
            input.verts[i * 3 + 2]
        ));
        vOrigMap[outVHandles[i]] = static_cast<int>(i);
    }

    for (size_t f = 0; f < nbFaces; ++f) {
        uint32_t v0 = input.faces[f * 4 + 0];
        uint32_t v1 = input.faces[f * 4 + 1];
        uint32_t v2 = input.faces[f * 4 + 2];
        uint32_t v3 = input.faces[f * 4 + 3];
        if (v0 >= nbVerts || v1 >= nbVerts || v2 >= nbVerts) continue;

        uint16_t gid = hasGroups ? input.faceGroups[f] : 0;

        if (v3 != 0xFFFFFFFFu && v3 < nbVerts) {
            // Split quad along shorter diagonal while preserving PolyGroup on both triangles
            const Point_3& p0 = mesh.point(outVHandles[v0]);
            const Point_3& p1 = mesh.point(outVHandles[v1]);
            const Point_3& p2 = mesh.point(outVHandles[v2]);
            const Point_3& p3 = mesh.point(outVHandles[v3]);
            double d02 = CGAL::squared_distance(p0, p2);
            double d13 = CGAL::squared_distance(p1, p3);

            Mesh::Face_index fh1, fh2;
            if (d02 <= d13) {
                fh1 = mesh.add_face(outVHandles[v0], outVHandles[v1], outVHandles[v2]);
                fh2 = mesh.add_face(outVHandles[v0], outVHandles[v2], outVHandles[v3]);
            } else {
                fh1 = mesh.add_face(outVHandles[v0], outVHandles[v1], outVHandles[v3]);
                fh2 = mesh.add_face(outVHandles[v1], outVHandles[v2], outVHandles[v3]);
            }
            if (fh1 == Mesh::null_face() || fh2 == Mesh::null_face()) {
                return false; // Non-manifold or inconsistent orientation -> fallback to soup repair
            }
            fGroupMap[fh1] = gid;
            fGroupMap[fh2] = gid;
        } else {
            Mesh::Face_index fh = mesh.add_face(outVHandles[v0], outVHandles[v1], outVHandles[v2]);
            if (fh == Mesh::null_face()) {
                return false; // Non-manifold or inconsistent orientation -> fallback to soup repair
            }
            fGroupMap[fh] = gid;
        }
    }
    return true;
}

void buildMeshFromSoup(
    const RawMeshData& input,
    Mesh& mesh,
    Mesh::Property_map<Mesh::Vertex_index, int>& vOrigMap,
    Mesh::Property_map<Mesh::Face_index, uint16_t>& fGroupMap)
{
    const size_t nbVerts = input.vertCount();
    const size_t nbFaces = input.faceCount();

    std::vector<CustomPoint> points(nbVerts);
    for (size_t i = 0; i < nbVerts; ++i) {
        points[i] = { input.verts[i * 3 + 0], input.verts[i * 3 + 1], input.verts[i * 3 + 2] };
    }

    std::vector<Polygon> polygons;
    polygons.reserve(nbFaces);
    for (size_t f = 0; f < nbFaces; ++f) {
        uint32_t v0 = input.faces[f * 4 + 0];
        uint32_t v1 = input.faces[f * 4 + 1];
        uint32_t v2 = input.faces[f * 4 + 2];
        uint32_t v3 = input.faces[f * 4 + 3];
        if (v0 >= nbVerts || v1 >= nbVerts || v2 >= nbVerts) continue;
        if (v3 != 0xFFFFFFFFu && v3 < nbVerts) {
            polygons.push_back({ v0, v1, v2, v3 });
        } else {
            polygons.push_back({ v0, v1, v2 });
        }
    }

    // Sequential soup repair & orientation (no data race)
    PMP::repair_polygon_soup(points, polygons, NP::geom_traits(ArrayTraits()));
    PMP::orient_polygon_soup(points, polygons);
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);

    for (auto v : mesh.vertices()) vOrigMap[v] = -1;
    for (auto f : mesh.faces()) fGroupMap[f] = 0;
}

} // namespace

namespace SPMeshHealer {

bool healMesh(const RawMeshData& input, RawMeshData& output, const HealerOptions& opts, HealerStats& stats) {
    stats = {};
    stats.inputVerts = input.vertCount();
    stats.inputFaces = input.faceCount();
    if (stats.inputVerts == 0 || stats.inputFaces == 0) return false;

    Mesh mesh;
    auto vOrigMap  = mesh.add_property_map<Mesh::Vertex_index, int>("v:orig_idx", -1).first;
    auto fGroupMap = mesh.add_property_map<Mesh::Face_index, uint16_t>("f:group_id", 0).first;
    std::vector<Mesh::Vertex_index> origVHandles;

    bool usedSoup = opts.repairSoup;
    if (!usedSoup) {
        if (!buildMeshDirect(input, mesh, vOrigMap, fGroupMap, origVHandles)) {
            if (!opts.quiet) {
                std::cout << "[SPMeshHealer] Non-manifold/inconsistent mesh detected, running polygon soup repair..." << std::endl;
            }
            mesh.clear();
            origVHandles.clear();
            vOrigMap  = mesh.add_property_map<Mesh::Vertex_index, int>("v:orig_idx", -1).first;
            fGroupMap = mesh.add_property_map<Mesh::Face_index, uint16_t>("f:group_id", 0).first;
            buildMeshFromSoup(input, mesh, vOrigMap, fGroupMap);
            usedSoup = true;
        }
    } else {
        buildMeshFromSoup(input, mesh, vOrigMap, fGroupMap);
    }

    const bool useKeepQuads = opts.keepQuads && !usedSoup && !opts.removeFragments && !origVHandles.empty();

    // Triangulate any remaining ngons (e.g. from soup repair)
    if (!CGAL::is_triangle_mesh(mesh)) {
        PMP::triangulate_faces(mesh);
    }

    // Pre-cleanup: stitch unwelded duplicate borders.
    // When keeping original quads intact, avoid deleting/collapsing faces or edges
    // before hole filling so original quads never go out of sync with patch faces.
    PMP::stitch_borders(mesh);
    if (!useKeepQuads) {
        PMP::remove_degenerate_edges(mesh, NP::geom_traits(Kernel()));
        PMP::remove_almost_degenerate_faces(mesh, NP::geom_traits(Kernel()));
        PMP::remove_degenerate_faces(mesh, NP::geom_traits(Kernel()));
        PMP::remove_isolated_vertices(mesh);
    }

    // Extract unique boundary cycles (O(total_border_halfedges))
    std::vector<Mesh::Halfedge_index> rawHoles;
    PMP::extract_boundary_cycles(mesh, std::back_inserter(rawHoles));

    std::vector<Mesh::Halfedge_index> holes;
    std::unordered_set<uint32_t> seenBorderHalfedges;
    size_t maxHoleVerts = 0;
    for (auto h : rawHoles) {
        if (!CGAL::is_border(h, mesh)) continue;
        if (!seenBorderHalfedges.insert(static_cast<uint32_t>(h)).second) continue;
        size_t k = 0;
        Mesh::Halfedge_index cur = h;
        do {
            seenBorderHalfedges.insert(static_cast<uint32_t>(cur));
            ++k;
            cur = mesh.next(cur);
        } while (cur != h);
        if (k > maxHoleVerts) maxHoleVerts = k;
        holes.push_back(h);
    }

    stats.holesFound = holes.size();
    if (!opts.quiet) {
        std::cout << "[SPMeshHealer] After stitch_borders: " << holes.size()
                  << " boundary hole(s), maxLoopVerts=" << maxHoleVerts << std::endl;
    }

    Mesh::Halfedge_index outerHull;
    bool hasOuterHull = false;
    if (opts.ignoreLargestHole && !holes.empty()) {
        double maxDiagSq = -1.0;
        for (auto h : holes) {
            double d = computeHoleBBoxDiagSq(mesh, h);
            if (d > maxDiagSq) {
                maxDiagSq = d;
                outerHull = h;
                hasOuterHull = true;
            }
        }
    }

    uint16_t maxGroup = 0;
    for (auto f : mesh.faces()) {
        if (fGroupMap[f] > maxGroup) maxGroup = fGroupMap[f];
    }
    const uint16_t fillGroup = static_cast<uint16_t>(maxGroup + 1);

    auto fIsPatch = mesh.add_property_map<Mesh::Face_index, uint8_t>("f:is_patch", 0).first;

    std::vector<uint8_t> holeDone(holes.size(), 0);

    // Parallel pass: extract isolated 2-ring local submeshes for small/medium holes and solve in parallel
    {
        struct LocalHoleJob {
            size_t holeIdx = 0;
            size_t bCount = 0;
            uint32_t hSrcLocal = 0;
            uint32_t hTgtLocal = 0;
            std::vector<Point_3> pts;
            std::vector<Mesh::Vertex_index> globalV;
            std::vector<std::array<uint32_t, 3>> tris;
        };
        struct LocalHoleResult {
            bool ok = false;
            std::vector<Point_3> newVerts;
            std::vector<std::array<int32_t, 3>> newTris; // >= 0: index into job.globalV; < 0: -1 - index into newVerts
        };

        const uint32_t maxV = mesh.number_of_vertices() + mesh.number_of_removed_vertices();
        const uint32_t maxF = mesh.number_of_faces() + mesh.number_of_removed_faces();
        std::vector<uint8_t> usedBoundaryVert(maxV, 0);
        std::vector<uint32_t> vStamp(maxV, 0);
        std::vector<uint32_t> fStamp(maxF, 0);
        std::vector<uint32_t> vLocalIdx(maxV, 0);
        uint32_t stamp = 0;

        std::vector<LocalHoleJob> jobs;
        jobs.reserve(holes.size());

        std::vector<Mesh::Vertex_index> bVerts;
        std::vector<Mesh::Vertex_index> ring1;
        std::vector<Mesh::Face_index> localFaces;

        for (size_t hi = 0; hi < holes.size(); ++hi) {
            auto h = holes[hi];
            if (hasOuterHull && h == outerHull) continue;
            if (!CGAL::is_border(h, mesh)) continue;

            bVerts.clear();
            bool skip = false;
            Mesh::Halfedge_index cur = h;
            do {
                auto v = mesh.target(cur);
                uint32_t vid = static_cast<uint32_t>(v);
                if (vid >= maxV || usedBoundaryVert[vid]) {
                    skip = true;
                    break;
                }
                bVerts.push_back(v);
                cur = mesh.next(cur);
                if (bVerts.size() > 512) {
                    skip = true;
                    break;
                }
            } while (cur != h);

            if (skip || bVerts.size() < 3) continue;
            if (opts.maxHoleVerts > 0 && static_cast<int>(bVerts.size()) > opts.maxHoleVerts) continue;

            // Check for pinch vertices (same vertex appears twice on this boundary loop)
            ++stamp;
            for (auto v : bVerts) {
                uint32_t vid = static_cast<uint32_t>(v);
                if (vStamp[vid] == stamp) { skip = true; break; }
                vStamp[vid] = stamp;
            }
            if (skip) continue;

            for (auto v : bVerts) usedBoundaryVert[static_cast<uint32_t>(v)] = 1;

            // Collect 1-ring vertices around boundary, then all faces incident to 1-ring (full 2-ring)
            ring1 = bVerts;
            for (auto bv : bVerts) {
                for (auto nv : CGAL::vertices_around_target(mesh.halfedge(bv), mesh)) {
                    uint32_t nvid = static_cast<uint32_t>(nv);
                    if (nvid < maxV && vStamp[nvid] != stamp) {
                        vStamp[nvid] = stamp;
                        ring1.push_back(nv);
                    }
                }
            }

            ++stamp; // new stamp for building local vertex/face indices
            localFaces.clear();
            for (auto rv : ring1) {
                for (auto f : CGAL::faces_around_target(mesh.halfedge(rv), mesh)) {
                    if (!f.is_valid() || f == Mesh::null_face()) continue;
                    uint32_t fid = static_cast<uint32_t>(f);
                    if (fid < maxF && fStamp[fid] != stamp) {
                        fStamp[fid] = stamp;
                        localFaces.push_back(f);
                    }
                }
            }
            if (localFaces.empty()) continue;

            LocalHoleJob job;
            job.holeIdx = hi;
            job.bCount = bVerts.size();
            job.tris.reserve(localFaces.size());

            auto getOrAddLocalV = [&](Mesh::Vertex_index gv) -> uint32_t {
                uint32_t gvid = static_cast<uint32_t>(gv);
                if (vStamp[gvid] != stamp) {
                    vStamp[gvid] = stamp;
                    uint32_t lid = static_cast<uint32_t>(job.pts.size());
                    vLocalIdx[gvid] = lid;
                    job.pts.push_back(mesh.point(gv));
                    job.globalV.push_back(gv);
                    return lid;
                }
                return vLocalIdx[gvid];
            };

            for (auto f : localFaces) {
                uint32_t tri[3];
                int k = 0;
                for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
                    if (k < 3) tri[k] = getOrAddLocalV(v);
                    ++k;
                }
                if (k == 3) job.tris.push_back({ tri[0], tri[1], tri[2] });
            }

            job.hSrcLocal = getOrAddLocalV(mesh.source(h));
            job.hTgtLocal = getOrAddLocalV(mesh.target(h));
            jobs.push_back(std::move(job));
        }

        std::vector<LocalHoleResult> results(jobs.size());

        #pragma omp parallel for schedule(dynamic, 4) if(jobs.size() > 8)
        for (int ji = 0; ji < static_cast<int>(jobs.size()); ++ji) {
            const auto& job = jobs[ji];
            auto& res = results[ji];

            Mesh localMesh;
            std::vector<Mesh::Vertex_index> lvHandles(job.pts.size());
            for (size_t i = 0; i < job.pts.size(); ++i) {
                lvHandles[i] = localMesh.add_vertex(job.pts[i]);
            }
            for (const auto& t : job.tris) {
                if (localMesh.add_face(lvHandles[t[0]], lvHandles[t[1]], lvHandles[t[2]]) == Mesh::null_face()) {
                    res.ok = false;
                    break;
                }
            }
            if (localMesh.number_of_faces() != job.tris.size()) continue;

            Mesh::Halfedge_index localH = localMesh.halfedge(lvHandles[job.hSrcLocal], lvHandles[job.hTgtLocal]);
            if (!localH.is_valid() || !CGAL::is_border(localH, localMesh)) continue;

            // Verify boundary loop length matches original hole
            size_t bLen = 0;
            Mesh::Halfedge_index lcur = localH;
            do {
                ++bLen;
                lcur = localMesh.next(lcur);
                if (bLen > job.bCount) break;
            } while (lcur != localH);
            if (bLen != job.bCount) continue;

            std::vector<Mesh::Face_index> patchFaces;
            std::vector<Mesh::Vertex_index> patchVerts;
            auto [ok, fo, vo] = PMP::triangulate_refine_and_fair_hole(
                localMesh, localH,
                NP::face_output_iterator(std::back_inserter(patchFaces))
                   .vertex_output_iterator(std::back_inserter(patchVerts))
                   .fairing_continuity(1)
            );
            if (!ok && patchFaces.empty() && CGAL::is_border(localH, localMesh)) {
                PMP::triangulate_and_refine_hole(
                    localMesh, localH,
                    NP::face_output_iterator(std::back_inserter(patchFaces))
                       .vertex_output_iterator(std::back_inserter(patchVerts))
                );
                ok = !patchFaces.empty();
            }
            if (!ok && patchFaces.empty()) continue;

            auto lvMap = localMesh.add_property_map<Mesh::Vertex_index, int32_t>("v:res_idx", 0).first;
            for (size_t i = 0; i < lvHandles.size(); ++i) {
                lvMap[lvHandles[i]] = static_cast<int32_t>(i);
            }
            res.newVerts.reserve(patchVerts.size());
            for (auto pv : patchVerts) {
                if (localMesh.is_valid(pv)) {
                    int32_t idx = -1 - static_cast<int32_t>(res.newVerts.size());
                    lvMap[pv] = idx;
                    res.newVerts.push_back(localMesh.point(pv));
                }
            }
            res.newTris.reserve(patchFaces.size());
            for (auto pf : patchFaces) {
                if (!localMesh.is_valid(pf)) continue;
                std::array<int32_t, 3> tri{};
                int k = 0;
                for (auto v : CGAL::vertices_around_face(localMesh.halfedge(pf), localMesh)) {
                    if (k < 3) tri[k] = lvMap[v];
                    ++k;
                }
                if (k == 3) res.newTris.push_back(tri);
            }
            res.ok = !res.newTris.empty();
        }

        // Merge parallel patch results back into main mesh
        std::vector<Mesh::Vertex_index> addedV;
        for (size_t ji = 0; ji < jobs.size(); ++ji) {
            const auto& job = jobs[ji];
            const auto& res = results[ji];
            if (!res.ok) continue;

            addedV.resize(res.newVerts.size());
            for (size_t i = 0; i < res.newVerts.size(); ++i) {
                auto nv = mesh.add_vertex(res.newVerts[i]);
                vOrigMap[nv] = -1;
                addedV[i] = nv;
            }

            bool allAdded = true;
            for (const auto& t : res.newTris) {
                auto v0 = (t[0] >= 0) ? job.globalV[t[0]] : addedV[-1 - t[0]];
                auto v1 = (t[1] >= 0) ? job.globalV[t[1]] : addedV[-1 - t[1]];
                auto v2 = (t[2] >= 0) ? job.globalV[t[2]] : addedV[-1 - t[2]];
                auto nf = mesh.add_face(v0, v1, v2);
                if (nf == Mesh::null_face()) {
                    allAdded = false;
                } else {
                    fGroupMap[nf] = fillGroup;
                    fIsPatch[nf] = 1;
                }
            }
            if (allAdded) {
                holeDone[job.holeIdx] = 1;
                ++stats.holesFilled;
            }
        }
    }

    for (size_t hi = 0; hi < holes.size(); ++hi) {
        if (holeDone[hi]) continue;
        auto h = holes[hi];
        if (hasOuterHull && h == outerHull) continue;
        if (!CGAL::is_border(h, mesh)) continue;

        if (opts.maxHoleVerts > 0) {
            int k = 0;
            Mesh::Halfedge_index cur = h;
            do { ++k; cur = mesh.next(cur); } while (cur != h);
            if (k > opts.maxHoleVerts) continue;
        }

        if (!opts.quiet && hi > 0 && (hi % 1000 == 0)) {
            std::cout << "[SPMeshHealer] Progress: " << hi << "/" << holes.size() << " holes..." << std::endl;
        }

        std::vector<Mesh::Face_index> patchFaces;
        std::vector<Mesh::Vertex_index> patchVerts;
        auto [ok, fo, vo] = PMP::triangulate_refine_and_fair_hole(
            mesh, h,
            NP::face_output_iterator(std::back_inserter(patchFaces))
               .vertex_output_iterator(std::back_inserter(patchVerts))
               .fairing_continuity(1)
        );
        if (!ok && patchFaces.empty() && CGAL::is_border(h, mesh)) {
            PMP::triangulate_and_refine_hole(
                mesh, h,
                NP::face_output_iterator(std::back_inserter(patchFaces))
                   .vertex_output_iterator(std::back_inserter(patchVerts))
            );
            ok = !patchFaces.empty();
        }
        if (!ok && patchFaces.empty() && CGAL::is_border(h, mesh)) {
            PMP::triangulate_hole(mesh, h, NP::face_output_iterator(std::back_inserter(patchFaces)));
            ok = !patchFaces.empty();
        }
        if (ok || !patchFaces.empty()) {
            ++stats.holesFilled;
            for (auto pf : patchFaces) {
                if (mesh.is_valid(pf)) {
                    fGroupMap[pf] = fillGroup;
                    fIsPatch[pf] = 1;
                }
            }
            for (auto pv : patchVerts) {
                if (mesh.is_valid(pv)) vOrigMap[pv] = -1;
            }
        }
    }

    // Second pass: catch any remaining small boundary loops (e.g., split loops or missed borders)
    {
        std::vector<Mesh::Halfedge_index> remRaw;
        PMP::extract_boundary_cycles(mesh, std::back_inserter(remRaw));
        for (auto h : remRaw) {
            if (!CGAL::is_border(h, mesh)) continue;
            if (hasOuterHull && h == outerHull) continue;
            if (opts.ignoreLargestHole) {
                // If outerHull halfedge handle changed or was split, skip the largest loop by bbox
                // (outerHull is already checked above, but guard against any large hull loop)
                double d = computeHoleBBoxDiagSq(mesh, h);
                if (hasOuterHull && d >= computeHoleBBoxDiagSq(mesh, outerHull) * 0.99) continue;
            }
            if (opts.maxHoleVerts > 0) {
                int k = 0;
                Mesh::Halfedge_index cur = h;
                do { ++k; cur = mesh.next(cur); } while (cur != h);
                if (k > opts.maxHoleVerts) continue;
            }
            std::vector<Mesh::Face_index> patchFaces;
            std::vector<Mesh::Vertex_index> patchVerts;
            PMP::triangulate_and_refine_hole(
                mesh, h,
                NP::face_output_iterator(std::back_inserter(patchFaces))
                   .vertex_output_iterator(std::back_inserter(patchVerts))
            );
            if (patchFaces.empty() && CGAL::is_border(h, mesh)) {
                PMP::triangulate_hole(mesh, h, NP::face_output_iterator(std::back_inserter(patchFaces)));
            }
            if (!patchFaces.empty()) {
                ++stats.holesFilled;
                for (auto pf : patchFaces) {
                    if (mesh.is_valid(pf)) {
                        fGroupMap[pf] = fillGroup;
                        fIsPatch[pf] = 1;
                    }
                }
                for (auto pv : patchVerts) {
                    if (mesh.is_valid(pv)) vOrigMap[pv] = -1;
                }
            }
        }
    }

    // Optional small fragment removal
    if (opts.removeFragments && mesh.number_of_faces() > 0) {
        auto fccmap = mesh.add_property_map<Mesh::Face_index, std::size_t>("f:cc", 0).first;
        std::size_t numCC = PMP::connected_components(mesh, fccmap);
        if (numCC > 1) {
            std::vector<std::size_t> ccSizes(numCC, 0);
            for (auto f : mesh.faces()) ccSizes[fccmap[f]]++;

            std::size_t largestCC = 0;
            for (std::size_t i = 1; i < numCC; ++i) {
                if (ccSizes[i] > ccSizes[largestCC]) largestCC = i;
            }

            const std::size_t totalFaces = mesh.number_of_faces();
            const std::size_t threshold = std::max<std::size_t>(
                static_cast<std::size_t>(std::max(1, opts.minFragmentFaces)),
                static_cast<std::size_t>(std::round(static_cast<double>(totalFaces) * opts.minFragmentFraction))
            );

            std::vector<std::size_t> smallCCs;
            for (std::size_t i = 0; i < numCC; ++i) {
                if (i != largestCC && ccSizes[i] < threshold) {
                    smallCCs.push_back(i);
                    stats.fragmentsRemoved += ccSizes[i];
                }
            }
            if (!smallCCs.empty()) {
                PMP::remove_connected_components(mesh, smallCCs, fccmap);
            }
        }
    }

    // Orientation check (only reliable on closed meshes)
    if (opts.fixOrientation && CGAL::is_closed(mesh) && !PMP::is_outward_oriented(mesh)) {
        PMP::reverse_face_orientations(mesh);
        stats.orientationFlipped = true;
    }

    // Post-cleanup: only remove isolated vertices (never delete faces after hole filling,
    // as that can punch small holes back into patches or desync keepQuads).
    PMP::remove_isolated_vertices(mesh);

    // Extract back to RawMeshData
    output.meshName = input.meshName;

    const bool hasInputMask  = !usedSoup && (input.vertMask.size() == stats.inputVerts);
    const bool hasInputPaint = !usedSoup && (input.vertPaint.size() == stats.inputVerts * 4);

    if (useKeepQuads) {
        // Keep original quads/tris intact and append only new patch triangles!
        const size_t outNbVerts = mesh.number_of_vertices();
        output.verts.resize(outNbVerts * 3);
        if (hasInputMask)  output.vertMask.resize(outNbVerts, 65535);
        if (hasInputPaint) output.vertPaint.resize(outNbVerts * 4, 255);

        std::unordered_map<Mesh::Vertex_index, uint32_t> vToOut;
        vToOut.reserve(outNbVerts);
        std::vector<uint32_t> origToOut(stats.inputVerts, 0xFFFFFFFFu);

        uint32_t vIdx = 0;
        for (auto v : mesh.vertices()) {
            const Point_3& p = mesh.point(v);
            output.verts[vIdx * 3 + 0] = static_cast<float>(p.x());
            output.verts[vIdx * 3 + 1] = static_cast<float>(p.y());
            output.verts[vIdx * 3 + 2] = static_cast<float>(p.z());

            int orig = vOrigMap[v];
            if (orig >= 0 && static_cast<size_t>(orig) < stats.inputVerts) {
                origToOut[orig] = vIdx;
            }
            if (hasInputMask && orig >= 0 && static_cast<size_t>(orig) < input.vertMask.size()) {
                output.vertMask[vIdx] = input.vertMask[orig];
            }
            if (hasInputPaint && orig >= 0 && static_cast<size_t>(orig) < stats.inputVerts) {
                output.vertPaint[vIdx * 4 + 0] = input.vertPaint[orig * 4 + 0];
                output.vertPaint[vIdx * 4 + 1] = input.vertPaint[orig * 4 + 1];
                output.vertPaint[vIdx * 4 + 2] = input.vertPaint[orig * 4 + 2];
                output.vertPaint[vIdx * 4 + 3] = input.vertPaint[orig * 4 + 3];
            } else if (hasInputPaint) {
                output.vertPaint[vIdx * 4 + 3] = 0;
            }
            vToOut[v] = vIdx++;
        }

        // Resolve any original vertices that were merged by stitch_borders using exact coordinates
        bool anyUnmapped = false;
        for (size_t i = 0; i < stats.inputVerts; ++i) {
            if (origToOut[i] == 0xFFFFFFFFu) { anyUnmapped = true; break; }
        }
        if (anyUnmapped) {
            struct PosKey {
                float x, y, z;
                bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
            };
            struct PosHash {
                size_t operator()(const PosKey& k) const {
                    uint32_t hx, hy, hz;
                    std::memcpy(&hx, &k.x, 4);
                    std::memcpy(&hy, &k.y, 4);
                    std::memcpy(&hz, &k.z, 4);
                    return (static_cast<size_t>(hx) * 73856093u) ^
                           (static_cast<size_t>(hy) * 19349663u) ^
                           (static_cast<size_t>(hz) * 83492791u);
                }
            };
            std::unordered_map<PosKey, uint32_t, PosHash> posMap;
            posMap.reserve(outNbVerts);
            for (uint32_t vi = 0; vi < outNbVerts; ++vi) {
                posMap.emplace(PosKey{output.verts[vi * 3], output.verts[vi * 3 + 1], output.verts[vi * 3 + 2]}, vi);
            }
            for (size_t i = 0; i < stats.inputVerts; ++i) {
                if (origToOut[i] == 0xFFFFFFFFu) {
                    auto it = posMap.find(PosKey{input.verts[i * 3], input.verts[i * 3 + 1], input.verts[i * 3 + 2]});
                    if (it != posMap.end()) origToOut[i] = it->second;
                }
            }
        }

        const bool hasInputGroups = input.faceGroups.size() == stats.inputFaces;
        output.faces.reserve(stats.inputFaces * 4 + mesh.number_of_faces() * 4);
        if (opts.preservePolygroups) {
            output.faceGroups.reserve(stats.inputFaces + mesh.number_of_faces());
        } else {
            output.faceGroups.clear();
        }

        // Copy original faces (preserving quads & tris)
        for (size_t f = 0; f < stats.inputFaces; ++f) {
            uint32_t i0 = input.faces[f * 4 + 0];
            uint32_t i1 = input.faces[f * 4 + 1];
            uint32_t i2 = input.faces[f * 4 + 2];
            uint32_t i3 = input.faces[f * 4 + 3];
            if (i0 >= stats.inputVerts || i1 >= stats.inputVerts || i2 >= stats.inputVerts) continue;
            uint32_t o0 = origToOut[i0], o1 = origToOut[i1], o2 = origToOut[i2];
            if (o0 == 0xFFFFFFFFu || o1 == 0xFFFFFFFFu || o2 == 0xFFFFFFFFu) continue;
            if (o0 == o1 || o1 == o2 || o2 == o0) continue;

            uint32_t o3 = 0xFFFFFFFFu;
            if (i3 != 0xFFFFFFFFu && i3 < stats.inputVerts) {
                o3 = origToOut[i3];
                if (o3 == 0xFFFFFFFFu || o3 == o0 || o3 == o1 || o3 == o2) o3 = 0xFFFFFFFFu;
            }

            if (stats.orientationFlipped) {
                if (o3 != 0xFFFFFFFFu) {
                    output.faces.push_back(o0); output.faces.push_back(o3); output.faces.push_back(o2); output.faces.push_back(o1);
                } else {
                    output.faces.push_back(o0); output.faces.push_back(o2); output.faces.push_back(o1); output.faces.push_back(0xFFFFFFFFu);
                }
            } else {
                output.faces.push_back(o0); output.faces.push_back(o1); output.faces.push_back(o2); output.faces.push_back(o3);
            }
            if (opts.preservePolygroups) {
                output.faceGroups.push_back(hasInputGroups ? input.faceGroups[f] : 0);
            }
        }

        // Append only newly created patch faces
        for (auto f : mesh.faces()) {
            if (!fIsPatch[f]) continue;
            uint32_t tri[3] = {0, 0, 0};
            int k = 0;
            for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
                if (k < 3) tri[k] = vToOut[v];
                ++k;
            }
            if (k == 3) {
                output.faces.push_back(tri[0]);
                output.faces.push_back(tri[1]);
                output.faces.push_back(tri[2]);
                output.faces.push_back(0xFFFFFFFFu);
                if (opts.preservePolygroups) {
                    output.faceGroups.push_back(fGroupMap[f]);
                }
            }
        }

        stats.outputVerts = outNbVerts;
        stats.outputFaces = output.faceCount();
        return true;
    }

    mesh.collect_garbage();

    const size_t outNbVerts = mesh.number_of_vertices();
    const size_t outNbFaces = mesh.number_of_faces();

    output.verts.resize(outNbVerts * 3);
    output.faces.resize(outNbFaces * 4, 0xFFFFFFFFu);
    if (opts.preservePolygroups) {
        output.faceGroups.resize(outNbFaces, 0);
    } else {
        output.faceGroups.clear();
    }

    if (hasInputMask)  output.vertMask.resize(outNbVerts, 65535);
    if (hasInputPaint) output.vertPaint.resize(outNbVerts * 4, 255);

    std::unordered_map<Mesh::Vertex_index, uint32_t> vToOut;
    vToOut.reserve(outNbVerts);
    uint32_t vIdx = 0;
    for (auto v : mesh.vertices()) {
        const Point_3& p = mesh.point(v);
        output.verts[vIdx * 3 + 0] = static_cast<float>(p.x());
        output.verts[vIdx * 3 + 1] = static_cast<float>(p.y());
        output.verts[vIdx * 3 + 2] = static_cast<float>(p.z());

        int orig = vOrigMap[v];
        if (hasInputMask && orig >= 0 && static_cast<size_t>(orig) < input.vertMask.size()) {
            output.vertMask[vIdx] = input.vertMask[orig];
        }
        if (hasInputPaint && orig >= 0 && static_cast<size_t>(orig) < stats.inputVerts) {
            output.vertPaint[vIdx * 4 + 0] = input.vertPaint[orig * 4 + 0];
            output.vertPaint[vIdx * 4 + 1] = input.vertPaint[orig * 4 + 1];
            output.vertPaint[vIdx * 4 + 2] = input.vertPaint[orig * 4 + 2];
            output.vertPaint[vIdx * 4 + 3] = input.vertPaint[orig * 4 + 3];
        } else if (hasInputPaint) {
            output.vertPaint[vIdx * 4 + 3] = 0;
        }
        vToOut[v] = vIdx++;
    }

    size_t fIdx = 0;
    for (auto f : mesh.faces()) {
        int k = 0;
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
            if (k < 4) output.faces[fIdx * 4 + k] = vToOut[v];
            ++k;
        }
        if (opts.preservePolygroups) {
            output.faceGroups[fIdx] = fGroupMap[f];
        }
        ++fIdx;
    }

    stats.outputVerts = outNbVerts;
    stats.outputFaces = outNbFaces;
    return true;
}

} // namespace SPMeshHealer
