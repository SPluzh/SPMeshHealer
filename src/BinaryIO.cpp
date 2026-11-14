#include "BinaryIO.h"
#include <fstream>
#include <cstring>

namespace SPMeshHealer {

namespace {
constexpr uint32_t SPBIN_MAGIC = 0x4E425053u; // "SPBN"
constexpr uint32_t SPBIN_VERSION = 1;
}

bool readBinary(const std::string& path, RawMeshData& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return false;

    uint32_t magic = 0, version = 0, nbVerts = 0, nbFaces = 0, flags = 0;
    in.read(reinterpret_cast<char*>(&magic), 4);
    in.read(reinterpret_cast<char*>(&version), 4);
    in.read(reinterpret_cast<char*>(&nbVerts), 4);
    in.read(reinterpret_cast<char*>(&nbFaces), 4);
    in.read(reinterpret_cast<char*>(&flags), 4);
    if (!in.good() || magic != SPBIN_MAGIC || version != SPBIN_VERSION) return false;

    out.verts.resize(static_cast<size_t>(nbVerts) * 3);
    out.faces.resize(static_cast<size_t>(nbFaces) * 4);
    if (nbVerts > 0) in.read(reinterpret_cast<char*>(out.verts.data()), out.verts.size() * sizeof(float));
    if (nbFaces > 0) in.read(reinterpret_cast<char*>(out.faces.data()), out.faces.size() * sizeof(uint32_t));

    if (flags & 1u) {
        out.faceGroups.resize(nbFaces);
        if (nbFaces > 0) in.read(reinterpret_cast<char*>(out.faceGroups.data()), out.faceGroups.size() * sizeof(uint16_t));
    }
    if (flags & 2u) {
        out.vertMask.resize(nbVerts);
        if (nbVerts > 0) in.read(reinterpret_cast<char*>(out.vertMask.data()), out.vertMask.size() * sizeof(uint16_t));
    }
    if (flags & 4u) {
        out.vertPaint.resize(static_cast<size_t>(nbVerts) * 4);
        if (nbVerts > 0) in.read(reinterpret_cast<char*>(out.vertPaint.data()), out.vertPaint.size());
    }
    return in.good();
}

bool writeBinary(const std::string& path, const RawMeshData& data) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;

    uint32_t nbVerts = static_cast<uint32_t>(data.vertCount());
    uint32_t nbFaces = static_cast<uint32_t>(data.faceCount());
    uint32_t flags = 0;
    if (data.faceGroups.size() == nbFaces) flags |= 1u;
    if (data.vertMask.size() == nbVerts) flags |= 2u;
    if (data.vertPaint.size() == static_cast<size_t>(nbVerts) * 4) flags |= 4u;

    out.write(reinterpret_cast<const char*>(&SPBIN_MAGIC), 4);
    out.write(reinterpret_cast<const char*>(&SPBIN_VERSION), 4);
    out.write(reinterpret_cast<const char*>(&nbVerts), 4);
    out.write(reinterpret_cast<const char*>(&nbFaces), 4);
    out.write(reinterpret_cast<const char*>(&flags), 4);

    if (nbVerts > 0) out.write(reinterpret_cast<const char*>(data.verts.data()), data.verts.size() * sizeof(float));
    if (nbFaces > 0) out.write(reinterpret_cast<const char*>(data.faces.data()), data.faces.size() * sizeof(uint32_t));
    if (flags & 1u && nbFaces > 0) out.write(reinterpret_cast<const char*>(data.faceGroups.data()), data.faceGroups.size() * sizeof(uint16_t));
    if (flags & 2u && nbVerts > 0) out.write(reinterpret_cast<const char*>(data.vertMask.data()), data.vertMask.size() * sizeof(uint16_t));
    if (flags & 4u && nbVerts > 0) out.write(reinterpret_cast<const char*>(data.vertPaint.data()), data.vertPaint.size());
    return out.good();
}

} // namespace SPMeshHealer
