#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct RawMeshData {
    // 3 floats per vertex (x, y, z)
    std::vector<float> verts;
    // 4 uint32_t per face: (v0, v1, v2, v3) where v3 == 0xFFFFFFFFu for triangles
    std::vector<uint32_t> faces;
    // Optional per-face PolyGroup ID (size == faceCount or empty)
    std::vector<uint16_t> faceGroups;
    // Optional per-vertex mask (0..65535, 65535 = unmasked)
    std::vector<uint16_t> vertMask;
    // Optional per-vertex BGRA paint (4 bytes per vertex)
    std::vector<uint8_t> vertPaint;
    // Optional mesh name (for GoZ)
    std::string meshName = "Mesh";

    size_t vertCount() const { return verts.size() / 3; }
    size_t faceCount() const { return faces.size() / 4; }
};

struct HealerOptions {
    bool ignoreLargestHole = false; // --hull 1
    int maxHoleVerts = 0;           // --max-hole-verts <N> (0 = unlimited)
    bool preservePolygroups = true; // --preserve-polygroups <0|1>
    bool keepQuads = true;          // --keep-quads <0|1> (preserve original quads on output)
    bool repairSoup = false;        // --repair-soup 1 (auto-enabled for STL)
    bool fixOrientation = true;     // check outward orientation when closed
    bool removeFragments = false;   // --remove-fragments
    int minFragmentFaces = 50;      // --min-faces
    float minFragmentFraction = 0.01f; // --min-frac
    bool quiet = false;             // --quiet
};

struct HealerStats {
    size_t inputVerts = 0;
    size_t inputFaces = 0;
    size_t holesFound = 0;
    size_t holesFilled = 0;
    size_t fragmentsRemoved = 0;
    size_t outputVerts = 0;
    size_t outputFaces = 0;
    bool orientationFlipped = false;
};
