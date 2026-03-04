#include "fivex/CollisionInfo.hpp"
#include "fivex/MeshShapeBuilder.hpp"
#include "fivex/PhiveWriter.hpp"
#include <cstring>
#include <cstdio>
#include <cstdarg>

static constexpr uint32_t byteswap32(uint32_t x) {
    return ((x << 24) & 0xFF000000) | ((x << 8) & 0x00FF0000) |
           ((x >> 8) & 0x0000FF00) | ((x >> 24) & 0x000000FF);
}

namespace FiveX {

template <typename T>
static bool jsonSafeGet(json& jason, const char* at, T* res) {
    if (jason.contains(at)) {
        auto obj = jason.at(at);
        if (!obj.is_null()) { *res = obj.get<T>(); return true; }
    }
    return false;
}

static std::vector<std::string> sMatNames;
static std::unordered_map<std::string, int> sMatNameMap;
static int getMatId(const char* matName) {
    auto it = sMatNameMap.find(matName);
    return it != sMatNameMap.end() ? it->second : 0;
}

static std::vector<std::string> sMatFlagNames;
static std::unordered_map<std::string, int> sMatFlagNameMap;
static int getMatFlagNameId(const char* flagName) {
    auto it = sMatFlagNameMap.find(flagName);
    return it != sMatFlagNameMap.end() ? it->second : -1;
}

static std::vector<std::string> sMatColFlagNames;
static std::unordered_map<std::string, int> sMatColFlagNameMap;
static int getMatColFlagNameId(const char* flagName) {
    auto it = sMatColFlagNameMap.find(flagName);
    return it != sMatColFlagNameMap.end() ? it->second : -1;
}

void CollisionInfo::setConfig(json& config) {
    sMatNames.clear(); sMatNameMap.clear();
    sMatFlagNames.clear(); sMatFlagNameMap.clear();
    sMatColFlagNames.clear(); sMatColFlagNameMap.clear();

    json matNames = {};
    jsonSafeGet(config, "mat_names", &matNames);
    for (auto& ita : matNames.items()) {
        sMatNames.push_back(ita.value().get<std::string>());
        sMatNameMap[sMatNames.back()] = (int)sMatNames.size() - 1;
    }

    json matFlagNames = {};
    jsonSafeGet(config, "mat_flag_names", &matFlagNames);
    for (auto& ita : matFlagNames.items()) {
        sMatFlagNames.push_back(ita.value().get<std::string>());
        sMatFlagNameMap[sMatFlagNames.back()] = (int)sMatFlagNames.size() - 1;
    }

    json matColFlagNames = {};
    jsonSafeGet(config, "col_disable_flag_names", &matColFlagNames);
    for (auto& ita : matColFlagNames.items()) {
        sMatColFlagNames.push_back(ita.value().get<std::string>());
        sMatColFlagNameMap[sMatColFlagNames.back()] = (int)sMatColFlagNames.size() - 1;
    }
}

// String utilities
namespace algorithm {
    inline void split(const std::string& in, std::vector<std::string>& out, std::string token) {
        out.clear();
        std::string temp;
        for (int i = 0; i < (int)in.size(); i++) {
            std::string test = in.substr(i, token.size());
            if (test == token) {
                if (!temp.empty()) { out.push_back(temp); temp.clear(); i += (int)token.size() - 1; }
                else out.push_back("");
            } else if (i + token.size() >= in.size()) {
                temp += in.substr(i, token.size());
                out.push_back(temp); break;
            } else temp += in[i];
        }
    }
    inline std::string tail(const std::string& in) {
        size_t ts = in.find_first_not_of(" \t");
        size_t ss = in.find_first_of(" \t", ts);
        size_t tails = in.find_first_not_of(" \t", ss);
        size_t taile = in.find_last_not_of(" \t");
        if (tails != std::string::npos && taile != std::string::npos) return in.substr(tails, taile - tails + 1);
        else if (tails != std::string::npos) return in.substr(tails);
        return "";
    }
    inline std::string firstToken(const std::string& in) {
        if (in.empty()) return "";
        size_t ts = in.find_first_not_of(" \t");
        size_t te = in.find_first_of(" \t", ts);
        if (ts != std::string::npos && te != std::string::npos) return in.substr(ts, te - ts);
        else if (ts != std::string::npos) return in.substr(ts);
        return "";
    }
}

CollisionInfo::CollisionInfo() {}
CollisionInfo::~CollisionInfo() { destroy(); }

void CollisionInfo::destroy() {
    if (!mInitialized) return;
    mInitialized = false;
    if (mOwnsGeometry) {
        delete[] mGeometry.m_vertices;
        delete[] mGeometry.m_triangles;
        mGeometry.m_vertices = nullptr;
        mGeometry.m_triangles = nullptr;
        mGeometry.m_numVertices = 0;
        mGeometry.m_numTriangles = 0;
        mOwnsGeometry = false;
    }
    mMaterials.clear();
}

bool CollisionInfo::initialize(u8* objData, u64 objSize, json& matConfig) {
    destroy();

    std::vector<const char*> lines;
    char* plainText = reinterpret_cast<char*>(objData);
    lines.push_back(plainText);
    for (char* line = strchr(plainText, '\n'); line != NULL; line = strchr(line, '\n')) {
        if (line > plainText && line[-1] == 0xD) line[-1] = '\0';
        *line = '\0';
        lines.push_back(++line);
    }

    std::map<std::string, int> mats;
    int curMatId = 0;

    struct Vec3 { float x, y, z; };
    std::vector<Vec3> vertices;
    std::vector<hkGeometry::Triangle> triangles;

    for (auto& ita : lines) {
        std::string line = ita;
        std::string first = algorithm::firstToken(line);

        if (first == "usemtl") {
            std::string matName = algorithm::tail(line);
            if (mats.find(matName) == mats.end()) {
                MaterialInfo info(0, 0, 0xFFFFFFFFFFFFFFFFULL);
                json matInfo;
                if (jsonSafeGet(matConfig, matName.c_str(), &matInfo)) {
                    std::string mn;
                    if (jsonSafeGet(matInfo, "mat_name", &mn)) info.mMatId = getMatId(mn.c_str());

                    json matArray;
                    jsonSafeGet(matInfo, "mat_flags", &matArray);
                    for (auto& f : matArray.items()) {
                        int id = getMatFlagNameId(f.value().get<std::string>().c_str());
                        if (id != -1) info.mMatFlags |= 1ULL << (u64)id;
                    }

                    json colArray;
                    jsonSafeGet(matInfo, "col_disable_flags", &colArray);
                    for (auto& f : colArray.items()) {
                        int id = getMatColFlagNameId(f.value().get<std::string>().c_str());
                        if (id != -1) info.mMatColFlags &= ~(1ULL << (u64)id);
                    }
                }
                mMaterials.push_back(info);
                mats[matName] = (int)mMaterials.size() - 1;
            }
            curMatId = mats[matName];
            continue;
        }

        if (first == "v") {
            std::vector<std::string> spos;
            algorithm::split(algorithm::tail(line), spos, " ");
            if (spos.size() >= 3)
                vertices.push_back({strtof(spos[0].c_str(), NULL), strtof(spos[1].c_str(), NULL), strtof(spos[2].c_str(), NULL)});
            continue;
        }

        if (first == "f") {
            if (mMaterials.empty()) mMaterials.push_back(MaterialInfo(0, 0, 0xFFFFFFFFFFFFFFFFULL));

            std::vector<std::string> sface, svert;
            algorithm::split(algorithm::tail(line), sface, " ");

            std::vector<int> idx(sface.size());
            for (int i = 0; i < (int)sface.size(); i++) {
                algorithm::split(sface[i], svert, "/");
                idx[i] = strtoul(svert[0].c_str(), NULL, 10) - 1;
            }

            for (int i = 1; i + 1 < (int)idx.size(); i++) {
                hkGeometry::Triangle t;
                t.m_a = idx[0]; t.m_b = idx[i]; t.m_c = idx[i + 1]; t.m_material = curMatId;
                triangles.push_back(t);
            }
            continue;
        }
    }

    if (vertices.empty() || triangles.empty()) return false;

    mGeometry.m_numVertices = (int)vertices.size();
    mGeometry.m_vertices = new hkVector4[vertices.size()];
    for (int i = 0; i < (int)vertices.size(); i++) {
        mGeometry.m_vertices[i].set(vertices[i].x, vertices[i].y, vertices[i].z, 0);
    }

    mGeometry.m_numTriangles = (int)triangles.size();
    mGeometry.m_triangles = new hkGeometry::Triangle[triangles.size()];
    memcpy(mGeometry.m_triangles, triangles.data(), sizeof(hkGeometry::Triangle) * triangles.size());

    mOwnsGeometry = true;
    mInitialized = true;
    return true;
}

bool CollisionInfo::initialize(Phive::PhiveMeshShape* shape) {
    destroy();

    MeshShapeBuilder::buildSurfaceGeometry(shape->mShape, mGeometry);

    for (int i = 0; i < shape->mMaterialNum; i++)
        mMaterials.push_back(MaterialInfo(shape->mMaterialArray[i].mMatId, shape->mMaterialArray[i].mFlags, shape->mMatColFlags[i]));

    mOwnsGeometry = true;
    mInitialized = true;
    return true;
}

void CollisionInfo::serializeToObj(std::vector<u8>& outObj, std::vector<u8>& outMatInfos) {
    outObj.reserve(4 * 1024 * 1024);

    auto writeStr = [&outObj](const char* fmt, ...) {
        char buf[512];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        size_t oldSize = outObj.size();
        outObj.resize(oldSize + len);
        memcpy(outObj.data() + oldSize, buf, len);
    };

    // Write vertices
    for (int i = 0; i < mGeometry.m_numVertices; i++)
        writeStr("v %.6f %.6f %.6f\n", mGeometry.m_vertices[i][0], mGeometry.m_vertices[i][1], mGeometry.m_vertices[i][2]);

    json matInfos = {};
    std::map<std::string, int> matNameUsedCnt;

    for (int m = 0; m < (int)mMaterials.size(); m++) {
        auto& mat = mMaterials[m];
        std::string matName = (mat.mMatId >= 0 && mat.mMatId < (int)sMatNames.size()) ? sMatNames[mat.mMatId] : "UNDEFINED";

        if (matNameUsedCnt.find(matName) == matNameUsedCnt.end()) matNameUsedCnt[matName] = 0;

        char matId[256];
        snprintf(matId, sizeof(matId), "%s%02i", matName.c_str(), matNameUsedCnt[matName]++);

        writeStr("o %s\n", matId);
        writeStr("usemtl %s\n", matId);

        for (int i = 0; i < mGeometry.m_numTriangles; i++) {
            if (mGeometry.m_triangles[i].m_material != m) continue;
            writeStr("f %i %i %i\n", mGeometry.m_triangles[i].m_a + 1, mGeometry.m_triangles[i].m_b + 1, mGeometry.m_triangles[i].m_c + 1);
        }

        matInfos[matId] = {
            {"mat_name", matName},
            {"mat_flags", json::array()},
            {"col_disable_flags", json::array()},
        };
        for (u64 i = 0; i < 64; i++) {
            if ((mat.mMatFlags & (1ULL << i)) != 0 && i < sMatFlagNames.size())
                matInfos[matId]["mat_flags"].push_back(sMatFlagNames[i]);
            if ((mat.mMatColFlags & (1ULL << i)) == 0 && i < sMatColFlagNames.size())
                matInfos[matId]["col_disable_flags"].push_back(sMatColFlagNames[i]);
        }
    }

    std::string matDataFile = matInfos.dump(4);
    outMatInfos.resize(matDataFile.length());
    memcpy(outMatInfos.data(), matDataFile.c_str(), matDataFile.length());
}

void CollisionInfo::serializeToBphsh(std::vector<u8>& out) {
    Phive::PhiveMeshShape* shape = buildShape();
    if (!shape) return;

    PhiveWriter writer;
    writer.serialize(shape);

    out = std::move(writer.mData);
    delete shape;
}

void CollisionInfo::loadFromBphsh(std::vector<u8>& in) {
    Phive::PhiveMeshShape* shape = new Phive::PhiveMeshShape();

    Phive::PhiveHeader& header = *reinterpret_cast<Phive::PhiveHeader*>(in.data());

    // Parse HKT to find DATA section
    u8* hkt = in.data() + header.HktOffset;
    u32 hktSize = byteswap32(*(u32*)hkt);

    // Skip TAG0 header (4 bytes size + "TAG0")
    // Skip flags (4 bytes)
    // Skip "SDKV" + version (12 bytes)
    // Find DATA section
    u8* pos = hkt + 4 + 4 + 4 + 4 + 8; // size + TAG0 + flags + SDKV + version
    u32 dataSize = byteswap32(*(u32*)pos) & 0x3FFFFFFF;
    pos += 4 + 4; // size + DATA tag

    hknpMeshShape* meshShape = reinterpret_cast<hknpMeshShape*>(pos);

    shape->mMaterialNum = (int)std::min((u64)header.TableSize0 / sizeof(Phive::PhiveShapeMaterialData),
                                         (u64)header.TableSize1 / sizeof(u64));
    shape->mMaterialArray = new Phive::PhiveShapeMaterialData[shape->mMaterialNum];
    memcpy(shape->mMaterialArray, in.data() + header.TableOffset0, sizeof(Phive::PhiveShapeMaterialData) * shape->mMaterialNum);

    shape->mMatColFlagsNum = shape->mMaterialNum;
    shape->mMatColFlags = new u64[shape->mMatColFlagsNum];
    memcpy(shape->mMatColFlags, in.data() + header.TableOffset1, sizeof(u64) * shape->mMatColFlagsNum);

    shape->mShape = meshShape;

    initialize(shape);

    // Don't delete shape->mShape since it points into the input buffer
    shape->mShape = nullptr;
    delete shape;
}

Phive::PhiveMeshShape* CollisionInfo::buildShape() {
    hknpMeshShape* meshShape = MeshShapeBuilder::build(mGeometry);
    if (!meshShape) return nullptr;

    Phive::PhiveMeshShape* shape = new Phive::PhiveMeshShape();

    shape->mMaterialNum = (int)mMaterials.size();
    shape->mMaterialArray = new Phive::PhiveShapeMaterialData[mMaterials.size()];
    for (int i = 0; i < (int)mMaterials.size(); i++)
        shape->mMaterialArray[i] = Phive::PhiveShapeMaterialData(mMaterials[i].mMatId, 0, mMaterials[i].mMatFlags);

    shape->mMatColFlagsNum = (int)mMaterials.size();
    shape->mMatColFlags = new u64[mMaterials.size()];
    for (int i = 0; i < (int)mMaterials.size(); i++)
        shape->mMatColFlags[i] = mMaterials[i].mMatColFlags;

    shape->mShape = meshShape;
    return shape;
}

}
