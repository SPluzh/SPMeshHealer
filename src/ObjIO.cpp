#include "ObjIO.h"

#include <charconv>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace SPMeshHealer {

namespace {

inline const char* skipWS(const char* p, const char* end) {
    while (p < end && (*p == ' ' || *p == '\t')) ++p;
    return p;
}

inline const char* parseFloatFast(const char* p, const char* end, float& outVal) {
    p = skipWS(p, end);
    if (p >= end) { outVal = 0.0f; return p; }
    auto res = std::from_chars(p, end, outVal);
    if (res.ec == std::errc()) return res.ptr;
    outVal = 0.0f;
    return p;
}

inline const char* parseIntFast(const char* p, const char* end, long& outVal, bool& ok) {
    p = skipWS(p, end);
    if (p >= end) { ok = false; return p; }
    auto res = std::from_chars(p, end, outVal);
    if (res.ec == std::errc()) { ok = true; return res.ptr; }
    ok = false;
    return p;
}

} // namespace

bool readOBJ(const std::string& path, RawMeshData& out) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in.is_open()) return false;

    const std::streamsize fileSize = in.tellg();
    if (fileSize <= 0) return false;
    in.seekg(0, std::ios::beg);

    std::string data;
    data.resize(static_cast<size_t>(fileSize));
    if (!in.read(data.data(), fileSize)) return false;
    in.close();

    // Rough heuristic reserve to avoid repeated reallocations
    const size_t estVerts = static_cast<size_t>(fileSize) / 55;
    out.verts.reserve(estVerts * 3);
    out.faces.reserve(estVerts * 4);
    out.faceGroups.reserve(estVerts);

    std::unordered_map<std::string, uint16_t> groupNameToId;
    uint16_t nextGroupId = 1;
    uint16_t currentGroup = 0;

    std::vector<uint32_t> polyIdx;
    polyIdx.reserve(8);

    const char* cur = data.data();
    const char* const end = cur + data.size();

    while (cur < end) {
        const char* nl = static_cast<const char*>(std::memchr(cur, '\n', static_cast<size_t>(end - cur)));
        const char* lineEnd = nl ? nl : end;
        if (lineEnd > cur && lineEnd[-1] == '\r') --lineEnd;

        const char* p = skipWS(cur, lineEnd);
        cur = nl ? (nl + 1) : end;

        if (p >= lineEnd || *p == '#') continue;

        if (*p == 'v' && p + 1 < lineEnd && (p[1] == ' ' || p[1] == '\t')) {
            p += 2;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            p = parseFloatFast(p, lineEnd, x);
            p = parseFloatFast(p, lineEnd, y);
            p = parseFloatFast(p, lineEnd, z);
            out.verts.push_back(x);
            out.verts.push_back(y);
            out.verts.push_back(z);
        } else if (*p == 'g' && p + 1 < lineEnd && (p[1] == ' ' || p[1] == '\t')) {
            p = skipWS(p + 2, lineEnd);
            const char* trimEnd = lineEnd;
            while (trimEnd > p && (trimEnd[-1] == ' ' || trimEnd[-1] == '\t')) --trimEnd;
            if (p >= trimEnd) {
                currentGroup = 0;
            } else {
                std::string gname(p, static_cast<size_t>(trimEnd - p));
                auto it = groupNameToId.find(gname);
                if (it == groupNameToId.end()) {
                    currentGroup = nextGroupId++;
                    groupNameToId.emplace(std::move(gname), currentGroup);
                } else {
                    currentGroup = it->second;
                }
            }
        } else if (*p == 'f' && p + 1 < lineEnd && (p[1] == ' ' || p[1] == '\t')) {
            polyIdx.clear();
            const int nbV = static_cast<int>(out.vertCount());
            p += 2;
            while (p < lineEnd) {
                p = skipWS(p, lineEnd);
                if (p >= lineEnd) break;
                long vIdx = 0;
                bool ok = false;
                p = parseIntFast(p, lineEnd, vIdx, ok);
                if (ok) {
                    if (vIdx > 0) vIdx -= 1;
                    else if (vIdx < 0) vIdx = nbV + vIdx;
                    if (vIdx >= 0 && vIdx < nbV) {
                        polyIdx.push_back(static_cast<uint32_t>(vIdx));
                    }
                }
                while (p < lineEnd && *p != ' ' && *p != '\t') ++p;
            }
            if (polyIdx.size() == 3) {
                out.faces.push_back(polyIdx[0]);
                out.faces.push_back(polyIdx[1]);
                out.faces.push_back(polyIdx[2]);
                out.faces.push_back(0xFFFFFFFFu);
                out.faceGroups.push_back(currentGroup);
            } else if (polyIdx.size() == 4) {
                out.faces.push_back(polyIdx[0]);
                out.faces.push_back(polyIdx[1]);
                out.faces.push_back(polyIdx[2]);
                out.faces.push_back(polyIdx[3]);
                out.faceGroups.push_back(currentGroup);
            } else if (polyIdx.size() > 4) {
                for (size_t i = 1; i + 1 < polyIdx.size(); ++i) {
                    out.faces.push_back(polyIdx[0]);
                    out.faces.push_back(polyIdx[i]);
                    out.faces.push_back(polyIdx[i + 1]);
                    out.faces.push_back(0xFFFFFFFFu);
                    out.faceGroups.push_back(currentGroup);
                }
            }
        }
    }

    return out.vertCount() > 0 && out.faceCount() > 0;
}

bool writeOBJ(const std::string& path, const RawMeshData& data) {
    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) return false;

    constexpr size_t BUF_SIZE = 4 * 1024 * 1024;
    std::vector<char> buf(BUF_SIZE);
    char* const base = buf.data();
    char* cur = base;
    char* const limit = base + BUF_SIZE - 256;

    auto flush = [&]() -> bool {
        if (cur > base) {
            size_t n = static_cast<size_t>(cur - base);
            if (std::fwrite(base, 1, n, fp) != n) return false;
            cur = base;
        }
        return true;
    };

    const char* header = "# Generated by SPMeshHealer\n";
    std::memcpy(cur, header, 28);
    cur += 28;

    const size_t nbVerts = data.vertCount();
    const size_t nbFaces = data.faceCount();
    const bool hasGroups = data.faceGroups.size() == nbFaces;
    constexpr int prec = std::numeric_limits<float>::max_digits10;

    for (size_t i = 0; i < nbVerts; ++i) {
        if (cur >= limit && !flush()) { std::fclose(fp); return false; }
        *cur++ = 'v';
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, data.verts[i * 3 + 0], std::chars_format::general, prec).ptr;
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, data.verts[i * 3 + 1], std::chars_format::general, prec).ptr;
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, data.verts[i * 3 + 2], std::chars_format::general, prec).ptr;
        *cur++ = '\n';
    }

    uint16_t lastGroup = 0xFFFFu;
    for (size_t f = 0; f < nbFaces; ++f) {
        if (cur >= limit && !flush()) { std::fclose(fp); return false; }

        if (hasGroups && data.faceGroups[f] != lastGroup) {
            lastGroup = data.faceGroups[f];
            std::memcpy(cur, "g group_", 8);
            cur += 8;
            cur = std::to_chars(cur, limit + 256, static_cast<uint32_t>(lastGroup)).ptr;
            *cur++ = '\n';
        }

        uint32_t v0 = data.faces[f * 4 + 0] + 1;
        uint32_t v1 = data.faces[f * 4 + 1] + 1;
        uint32_t v2 = data.faces[f * 4 + 2] + 1;
        uint32_t v3 = data.faces[f * 4 + 3];

        *cur++ = 'f';
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, v0).ptr;
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, v1).ptr;
        *cur++ = ' ';
        cur = std::to_chars(cur, limit + 256, v2).ptr;
        if (v3 != 0xFFFFFFFFu) {
            *cur++ = ' ';
            cur = std::to_chars(cur, limit + 256, v3 + 1).ptr;
        }
        *cur++ = '\n';
    }

    bool ok = flush();
    std::fclose(fp);
    return ok;
}

} // namespace SPMeshHealer
