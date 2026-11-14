#include "HealerTypes.h"
#include "HealerCore.h"
#include "BinaryIO.h"
#include "StlIO.h"
#include "ObjIO.h"
#include "GozIO.h"
#include "FbxIO.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace fs = std::filesystem;

namespace {

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

fs::path getExecutableDir(const char* argv0) {
#ifdef _WIN32
    char buf[MAX_PATH] = {};
    DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (len > 0 && len < MAX_PATH) {
        return fs::path(buf).parent_path();
    }
#endif
    if (argv0 && argv0[0] != '\0') {
        std::error_code ec;
        fs::path p = fs::absolute(fs::path(argv0), ec);
        if (!ec) return p.parent_path();
    }
    return fs::current_path();
}

fs::path getTempWorkDir() {
    std::error_code ec;
    fs::path tmp = fs::temp_directory_path(ec);
    if (ec) tmp = fs::current_path();
    fs::path dir = tmp / "SPMeshHealer";
    fs::create_directories(dir, ec);
    return dir;
}

class FileLogger {
public:
    explicit FileLogger(const fs::path& logPath) {
        std::error_code ec;
        if (logPath.has_parent_path()) {
            fs::create_directories(logPath.parent_path(), ec);
        }
        m_file.open(logPath, std::ios::app);
    }

    void write(const std::string& msg, bool toStdout = true, bool toStderr = false) {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tmLocal{};
#ifdef _WIN32
        localtime_s(&tmLocal, &tt);
#else
        localtime_r(&tt, &tmLocal);
#endif
        std::ostringstream oss;
        oss << '[' << std::put_time(&tmLocal, "%Y-%m-%d %H:%M:%S") << "] " << msg;
        std::string line = oss.str();

        if (m_file.is_open()) {
            m_file << line << '\n';
            m_file.flush();
        }
        if (toStderr) std::cerr << msg << std::endl;
        else if (toStdout) std::cout << msg << std::endl;
    }

private:
    std::ofstream m_file;
};

void loadConfigFile(const fs::path& cfgPath, HealerOptions& opts, std::string& cfgInPath, std::string& cfgOutPath) {
    std::ifstream in(cfgPath);
    if (!in.is_open()) return;

    std::string line;
    while (std::getline(in, line)) {
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
        if (line.empty() || line[0] == '#') continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = toLower(line.substr(0, eq));
        key.erase(std::remove_if(key.begin(), key.end(), [](unsigned char c) { return std::isspace(c); }), key.end());
        std::string val = line.substr(eq + 1);
        size_t first = val.find_first_not_of(" \t");
        if (first != std::string::npos) val = val.substr(first);
        else val.clear();

        try {
            if (key == "hull" || key == "ignorelargesthole") opts.ignoreLargestHole = (std::stoi(val) != 0);
            else if (key == "maxholeverts") opts.maxHoleVerts = std::stoi(val);
            else if (key == "preservepolygroups") opts.preservePolygroups = (std::stoi(val) != 0);
            else if (key == "keepquads") opts.keepQuads = (std::stoi(val) != 0);
            else if (key == "repairsoup") opts.repairSoup = (std::stoi(val) != 0);
            else if (key == "fixorientation") opts.fixOrientation = (std::stoi(val) != 0);
            else if (key == "removefragments" || key == "removesmallfragments") opts.removeFragments = (std::stoi(val) != 0);
            else if (key == "minfaces" || key == "fragmentminfaces") opts.minFragmentFaces = std::stoi(val);
            else if (key == "minfraction" || key == "fragmentminfraction") opts.minFragmentFraction = std::stof(val);
            else if (key == "infile") cfgInPath = val;
            else if (key == "outfile") cfgOutPath = val;
        } catch (...) {
        }
    }
}

enum class MeshFormat {
    Unknown,
    Binary,
    GoZ,
    OBJ,
    STL,
    FBX
};

MeshFormat detectFormat(const std::string& path, bool forceBin) {
    if (forceBin) return MeshFormat::Binary;
    std::string ext = toLower(fs::path(path).extension().string());
    if (ext == ".spbin" || ext == ".bin") return MeshFormat::Binary;
    if (ext == ".goz") return MeshFormat::GoZ;
    if (ext == ".obj") return MeshFormat::OBJ;
    if (ext == ".stl") return MeshFormat::STL;
    if (ext == ".fbx") return MeshFormat::FBX;
    return MeshFormat::Unknown;
}

void printUsage() {
    std::cout << "SPMeshHealer - Universal CGAL Mesh & Scan Repair Tool\n"
              << "Usage:\n"
              << "  SPMeshHealer --in <input> --out <output> [options]\n"
              << "  SPMeshHealer <input> <output> [options]\n\n"
              << "Supported formats: .goz, .obj, .stl, .spbin (--bin)\n\n"
              << "Options:\n"
              << "  --bin                 Force raw binary format (SPSculpt bridge)\n"
              << "  --hull <0|1>                 1 = ignore largest open boundary loop (default: 0)\n"
              << "  --max-hole-verts <N>         Skip holes with more than N boundary verts (0 = unlimited)\n"
              << "  --preserve-polygroups <0|1>  Preserve input PolyGroups & tag filled holes (default: 1)\n"
              << "  --keep-quads <0|1>           Preserve input quads/polygons, append only hole patches (default: 1)\n"
              << "  --repair-soup <0|1>          1 = force polygon soup repair (auto for .stl)\n"
              << "  --remove-fragments    Remove small disconnected islands\n"
              << "  --min-faces <N>       Min face count to keep an island (default: 50)\n"
              << "  --min-frac <F>        Min face fraction to keep an island (default: 0.01)\n"
              << "  --config <path>       Path to config.txt (default: next to executable)\n"
              << "  --quiet               Suppress console output\n";
}

} // namespace

int main(int argc, char* argv[]) {
    const fs::path exeDir = getExecutableDir(argc > 0 ? argv[0] : nullptr);
    fs::path configPath = exeDir / "config.txt";

    HealerOptions opts;
    std::string inPath;
    std::string outPath;
    std::string cfgInPath;
    std::string cfgOutPath;
    fs::path logPath = getTempWorkDir() / "SPMeshHealer.log";
    bool forceBin = false;
    bool hullExplicit = false;
    bool repairSoupExplicit = false;

    // First pass: check --config, --log, or a positional .txt config file
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) { configPath = argv[++i]; }
        else if (a == "--log" && i + 1 < argc) { logPath = argv[++i]; }
        else if (a == "--in" || a == "--out") { ++i; }
        else if (a[0] != '-' && a.size() > 4) {
            std::string tail = a.substr(a.size() - 4);
            for (char& c : tail) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (tail == ".txt") configPath = a;
        }
    }
    loadConfigFile(configPath, opts, cfgInPath, cfgOutPath);
    FileLogger logger(logPath);

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h" || arg == "--help") && argc == 2) {
            printUsage();
            return 0;
        } else if (arg == "--in" && i + 1 < argc) {
            inPath = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outPath = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            ++i;
        } else if (arg == "--log" && i + 1 < argc) {
            ++i;
        } else if (arg == "--bin") {
            forceBin = true;
        } else if (arg == "--hull" && i + 1 < argc) {
            opts.ignoreLargestHole = (std::stoi(argv[++i]) != 0);
            hullExplicit = true;
        } else if (arg == "--max-hole-verts" && i + 1 < argc) {
            opts.maxHoleVerts = std::stoi(argv[++i]);
        } else if (arg == "--preserve-polygroups" && i + 1 < argc) {
            opts.preservePolygroups = (std::stoi(argv[++i]) != 0);
        } else if (arg == "--keep-quads" && i + 1 < argc) {
            opts.keepQuads = (std::stoi(argv[++i]) != 0);
        } else if (arg == "--repair-soup" && i + 1 < argc) {
            opts.repairSoup = (std::stoi(argv[++i]) != 0);
            repairSoupExplicit = true;
        } else if (arg == "--remove-fragments") {
            opts.removeFragments = true;
        } else if (arg == "--min-faces" && i + 1 < argc) {
            opts.minFragmentFaces = std::stoi(argv[++i]);
        } else if (arg == "--min-frac" && i + 1 < argc) {
            opts.minFragmentFraction = std::stof(argv[++i]);
        } else if (arg == "--quiet") {
            opts.quiet = true;
        } else if (arg[0] != '-') {
            std::string tail = (arg.size() > 4) ? arg.substr(arg.size() - 4) : "";
            for (char& c : tail) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (tail != ".txt") {
                if (inPath.empty()) inPath = arg;
                else if (outPath.empty()) outPath = arg;
            }
        }
    }
    (void)hullExplicit;

    if (inPath.empty()) {
        if (!cfgInPath.empty()) {
            inPath = cfgInPath;
        } else {
            fs::path tmpDir = getTempWorkDir();
            inPath = (tmpDir / "input.stl").string();
        }
    }
    if (outPath.empty()) {
        if (!cfgOutPath.empty()) {
            outPath = cfgOutPath;
        } else {
            fs::path tmpDir = getTempWorkDir();
            outPath = (tmpDir / "output.stl").string();
        }
    }

    MeshFormat inFmt  = detectFormat(inPath, forceBin);
    MeshFormat outFmt = detectFormat(outPath, forceBin);
    if (inFmt == MeshFormat::Unknown || outFmt == MeshFormat::Unknown) {
        logger.write("[SPMeshHealer ERROR] Unsupported file format: " + inPath + " -> " + outPath, false, true);
        return 1;
    }

    if (inFmt == MeshFormat::STL && !repairSoupExplicit) {
        opts.repairSoup = true;
    }

    logger.write("[SPMeshHealer] Start | in=" + inPath + " | out=" + outPath
                 + " | hull=" + std::to_string(opts.ignoreLargestHole ? 1 : 0)
                 + " | polygroups=" + std::to_string(opts.preservePolygroups ? 1 : 0)
                 + " | keepQuads=" + std::to_string(opts.keepQuads ? 1 : 0)
                 + " | repairSoup=" + std::to_string(opts.repairSoup ? 1 : 0)
                 + " | removeFragments=" + std::to_string(opts.removeFragments ? 1 : 0),
                 !opts.quiet, false);

    auto t0 = std::chrono::high_resolution_clock::now();

    RawMeshData inputMesh;
    bool loaded = false;
    switch (inFmt) {
        case MeshFormat::Binary: loaded = SPMeshHealer::readBinary(inPath, inputMesh); break;
        case MeshFormat::GoZ:    loaded = SPMeshHealer::readGoZ(inPath, inputMesh); break;
        case MeshFormat::OBJ:    loaded = SPMeshHealer::readOBJ(inPath, inputMesh); break;
        case MeshFormat::STL:    loaded = SPMeshHealer::readSTL(inPath, inputMesh); break;
        case MeshFormat::FBX:    loaded = SPMeshHealer::readFBX(inPath, inputMesh); break;
        default: break;
    }

    if (!loaded) {
        logger.write("[SPMeshHealer ERROR] Failed to read input mesh: " + inPath, false, true);
        return 2;
    }

    auto tLoaded = std::chrono::high_resolution_clock::now();
    double msLoad = std::chrono::duration<double, std::milli>(tLoaded - t0).count();
    logger.write("[SPMeshHealer] Loaded in " + std::to_string(msLoad) + " ms | verts="
                 + std::to_string(inputMesh.vertCount()) + " faces=" + std::to_string(inputMesh.faceCount()),
                 !opts.quiet, false);

    RawMeshData outputMesh;
    HealerStats stats;
    if (!SPMeshHealer::healMesh(inputMesh, outputMesh, opts, stats)) {
        logger.write("[SPMeshHealer ERROR] Mesh healing failed.", false, true);
        return 3;
    }

    auto tHealed = std::chrono::high_resolution_clock::now();
    double msHeal = std::chrono::duration<double, std::milli>(tHealed - tLoaded).count();
    logger.write("[SPMeshHealer] Healed in " + std::to_string(msHeal) + " ms | holes="
                 + std::to_string(stats.holesFilled) + "/" + std::to_string(stats.holesFound),
                 !opts.quiet, false);

    const std::string tmpOutPath = outPath + ".tmp";
    bool saved = false;
    switch (outFmt) {
        case MeshFormat::Binary: saved = SPMeshHealer::writeBinary(tmpOutPath, outputMesh); break;
        case MeshFormat::GoZ:    saved = SPMeshHealer::writeGoZ(tmpOutPath, outputMesh); break;
        case MeshFormat::OBJ:    saved = SPMeshHealer::writeOBJ(tmpOutPath, outputMesh); break;
        case MeshFormat::STL:    saved = SPMeshHealer::writeSTL(tmpOutPath, outputMesh); break;
        case MeshFormat::FBX:    saved = SPMeshHealer::writeFBX(tmpOutPath, outputMesh); break;
        default: break;
    }

    if (saved) {
        std::error_code ec;
        fs::remove(outPath, ec);
        fs::rename(tmpOutPath, outPath, ec);
        if (ec) saved = false;
    }

    if (!saved) {
        logger.write("[SPMeshHealer ERROR] Failed to write output mesh: " + outPath, false, true);
        return 4;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::string summary = "[SPMeshHealer] Done in " + std::to_string(ms) + " ms | "
        + "Verts: " + std::to_string(stats.inputVerts) + " -> " + std::to_string(stats.outputVerts) + " | "
        + "Faces: " + std::to_string(stats.inputFaces) + " -> " + std::to_string(stats.outputFaces) + " | "
        + "Holes filled: " + std::to_string(stats.holesFilled) + "/" + std::to_string(stats.holesFound)
        + (stats.fragmentsRemoved > 0 ? " | Fragments removed: " + std::to_string(stats.fragmentsRemoved) : "")
        + (stats.orientationFlipped ? " | Orientation fixed" : "");
    logger.write(summary, !opts.quiet, false);

    return 0;
}
