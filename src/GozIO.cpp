#include "GozIO.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <vector>

namespace SPMeshHealer {

namespace {

constexpr uint32_t TAG_END         = 0;
constexpr uint32_t TAG_OBJECT_NAME = 1;
constexpr uint32_t TAG_MESH_NAME   = 0x1389;
constexpr uint32_t TAG_SUBDIVISION = 0x138A;
constexpr uint32_t TAG_VERTICES    = 0x2711;
constexpr uint32_t TAG_FACES       = 0x4E21;
constexpr uint32_t TAG_MASK        = 0x7532;
constexpr uint32_t TAG_PAINT       = 0x88B9;
constexpr uint32_t TAG_POLYGROUPS  = 0x9C41;

inline uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t readU64(const uint8_t* p) {
    return static_cast<uint64_t>(readU32(p)) |
           (static_cast<uint64_t>(readU32(p + 4)) << 32);
}

inline void writeU16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline void writeU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

inline void writeU64(uint8_t* p, uint64_t v) {
    writeU32(p, static_cast<uint32_t>(v & 0xFFFFFFFFULL));
    writeU32(p + 4, static_cast<uint32_t>((v >> 32) & 0xFFFFFFFFULL));
}

inline void writeChunkHeader(uint8_t* p, uint32_t tag, uint32_t chunkSize, uint64_t count) {
    writeU32(p + 0, tag);
    writeU32(p + 4, chunkSize);
    writeU64(p + 8, count);
}

} // namespace

bool readGoZ(const std::string& path, RawMeshData& out) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in.is_open()) return false;

    const std::streamsize fileSize = in.tellg();
    if (fileSize < 48) return false;
    in.seekg(0, std::ios::beg);

    std::vector<uint8_t> buf(static_cast<size_t>(fileSize));
    if (!in.read(reinterpret_cast<char*>(buf.data()), fileSize)) return false;

    if (std::memcmp(buf.data(), "GoZb", 4) != 0) return false;

    size_t pos = 32;
    while (pos + 16 <= buf.size()) {
        uint32_t tag = readU32(buf.data() + pos);
        uint32_t chunkSize = readU32(buf.data() + pos + 4);
        uint64_t count = readU64(buf.data() + pos + 8);
        if (tag == TAG_END) break;
        if (chunkSize < 16 || pos + chunkSize > buf.size()) break;

        const uint8_t* payload = buf.data() + pos + 16;
        const size_t payloadSize = chunkSize - 16;

        if (tag == TAG_OBJECT_NAME && payloadSize > 0) {
            size_t len = 0;
            while (len < payloadSize && payload[len] != '\0') ++len;
            out.meshName.assign(reinterpret_cast<const char*>(payload), len);
        } else if (tag == TAG_VERTICES && payloadSize >= count * 12) {
            out.verts.resize(static_cast<size_t>(count) * 3);
            std::memcpy(out.verts.data(), payload, static_cast<size_t>(count) * 12);
        } else if (tag == TAG_FACES && payloadSize >= count * 16) {
            out.faces.resize(static_cast<size_t>(count) * 4);
            std::memcpy(out.faces.data(), payload, static_cast<size_t>(count) * 16);
        } else if (tag == TAG_MASK && payloadSize >= count * 2) {
            out.vertMask.resize(static_cast<size_t>(count));
            for (size_t i = 0; i < count; ++i) {
                out.vertMask[i] = readU16(payload + i * 2);
            }
        } else if (tag == TAG_PAINT && payloadSize >= count * 4) {
            out.vertPaint.resize(static_cast<size_t>(count) * 4);
            std::memcpy(out.vertPaint.data(), payload, static_cast<size_t>(count) * 4);
        } else if (tag == TAG_POLYGROUPS && payloadSize >= count * 2) {
            out.faceGroups.resize(static_cast<size_t>(count));
            for (size_t i = 0; i < count; ++i) {
                out.faceGroups[i] = readU16(payload + i * 2);
            }
        }

        pos += chunkSize;
    }

    return out.vertCount() > 0 && out.faceCount() > 0;
}

bool writeGoZ(const std::string& path, const RawMeshData& data) {
    const size_t nbVerts = data.vertCount();
    const size_t nbFaces = data.faceCount();
    if (nbVerts == 0 || nbFaces == 0) return false;

    std::string objName = data.meshName.empty() ? "GoZMesh_SPMeshHealer" : data.meshName;
    size_t namePayloadLen = std::max<size_t>(16, ((objName.length() + 15) / 16) * 16);
    uint32_t objChunkSize = static_cast<uint32_t>(16 + namePayloadLen);

    uint32_t tagMeshNameChunkSize = 20;
    uint32_t tagSubdivChunkSize   = 32;
    uint32_t vertsPayloadLen      = static_cast<uint32_t>(nbVerts * 12);
    uint32_t vertsChunkSize       = 16 + vertsPayloadLen;
    uint32_t facesPayloadLen      = static_cast<uint32_t>(nbFaces * 16);
    uint32_t facesChunkSize       = 16 + facesPayloadLen;
    uint32_t paintPayloadLen      = static_cast<uint32_t>(nbVerts * 4);
    uint32_t paintChunkSize       = 16 + paintPayloadLen;
    uint32_t maskPayloadLen       = static_cast<uint32_t>(nbVerts * 2);
    uint32_t maskChunkSize        = 16 + maskPayloadLen;
    uint32_t groupsPayloadLen     = static_cast<uint32_t>(nbFaces * 2);
    uint32_t groupsChunkSize      = 16 + groupsPayloadLen;
    uint32_t endChunkSize         = 16;

    size_t totalBytes = 32 + objChunkSize + tagMeshNameChunkSize + tagSubdivChunkSize
                      + vertsChunkSize + facesChunkSize + paintChunkSize
                      + maskChunkSize + groupsChunkSize + endChunkSize;

    std::vector<uint8_t> buf(totalBytes);
    uint8_t* p = buf.data();

    static const char GOZ_MAGIC[32] = {
        'G', 'o', 'Z', 'b', ' ', '1', '.', '0', ' ', 'Z', 'B', 'r', 'u', 's', 'h', ' ',
        'G', 'o', 'Z', ' ', 'B', 'i', 'n', 'a', 'r', 'y', '.', '.', '.', '.', '.', '.'
    };
    std::memcpy(p, GOZ_MAGIC, 32);
    p += 32;

    writeChunkHeader(p, TAG_OBJECT_NAME, objChunkSize, 1);
    p += 16;
    std::memset(p, 0, namePayloadLen);
    std::memcpy(p, objName.data(), std::min(objName.length(), namePayloadLen));
    p += namePayloadLen;

    writeChunkHeader(p, TAG_MESH_NAME, tagMeshNameChunkSize, 1);
    p += 16;
    writeU32(p, 0x0806);
    p += 4;

    writeChunkHeader(p, TAG_SUBDIVISION, tagSubdivChunkSize, 1);
    p += 16;
    std::memset(p, 0, 16);
    p += 16;

    writeChunkHeader(p, TAG_VERTICES, vertsChunkSize, nbVerts);
    p += 16;
    std::memcpy(p, data.verts.data(), vertsPayloadLen);
    p += vertsPayloadLen;

    writeChunkHeader(p, TAG_FACES, facesChunkSize, nbFaces);
    p += 16;
    std::memcpy(p, data.faces.data(), facesPayloadLen);
    p += facesPayloadLen;

    writeChunkHeader(p, TAG_PAINT, paintChunkSize, nbVerts);
    p += 16;
    if (data.vertPaint.size() == paintPayloadLen) {
        std::memcpy(p, data.vertPaint.data(), paintPayloadLen);
    } else {
        for (size_t i = 0; i < nbVerts; ++i) {
            p[i * 4 + 0] = 255;
            p[i * 4 + 1] = 255;
            p[i * 4 + 2] = 255;
            p[i * 4 + 3] = 0;
        }
    }
    p += paintPayloadLen;

    writeChunkHeader(p, TAG_MASK, maskChunkSize, nbVerts);
    p += 16;
    if (data.vertMask.size() == nbVerts) {
        for (size_t i = 0; i < nbVerts; ++i) writeU16(p + i * 2, data.vertMask[i]);
    } else {
        for (size_t i = 0; i < nbVerts; ++i) writeU16(p + i * 2, 65535);
    }
    p += maskPayloadLen;

    writeChunkHeader(p, TAG_POLYGROUPS, groupsChunkSize, nbFaces);
    p += 16;
    if (data.faceGroups.size() == nbFaces) {
        for (size_t i = 0; i < nbFaces; ++i) writeU16(p + i * 2, data.faceGroups[i]);
    } else {
        std::memset(p, 0, groupsPayloadLen);
    }
    p += groupsPayloadLen;

    writeChunkHeader(p, TAG_END, 0, 0);

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;
    out.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    return out.good();
}

} // namespace SPMeshHealer
