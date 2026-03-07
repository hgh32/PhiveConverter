#include "fivex/AampFile.hpp"
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <sstream>
#include <functional>

namespace FiveX {

// CRC32 lookup table
static u32 sCrc32Table[256];
static bool sCrc32Init = false;

static void initCrc32() {
    if (sCrc32Init) return;
    for (u32 i = 0; i < 256; i++) {
        u32 crc = i;
        for (int j = 0; j < 8; j++)
            crc = (crc >> 1) ^ (crc & 1 ? 0xEDB88320 : 0);
        sCrc32Table[i] = crc;
    }
    sCrc32Init = true;
}

static u32 calcCrc32(const void* data, size_t size) {
    initCrc32();
    u32 crc = 0xFFFFFFFF;
    const u8* p = (const u8*)data;
    for (size_t i = 0; i < size; i++)
        crc = sCrc32Table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFF;
}

// Known hash dictionary for phcl AAMP parameters
static std::map<u32, std::string>& getDictionary() {
    static std::map<u32, std::string> dict;
    static bool initialized = false;
    if (!initialized) {
        const char* names[] = {
            // Structural names
            "param_root", "cloth_mesh_list", "collidable_list",
            // Indexed object names (cloth_mesh_N, collidable_N)
            "cloth_mesh_0", "cloth_mesh_1", "cloth_mesh_2", "cloth_mesh_3",
            "cloth_mesh_4", "cloth_mesh_5", "cloth_mesh_6", "cloth_mesh_7",
            "cloth_mesh_8", "cloth_mesh_9",
            "collidable_0", "collidable_1", "collidable_2", "collidable_3",
            "collidable_4", "collidable_5", "collidable_6", "collidable_7",
            "collidable_8", "collidable_9",
            // cloth_mesh object params
            "Name", "BaseBone", "WindPreset", "BoneCorrection",
            "BoneCorrectionAxisOrder", "Twist", "TwistSwingAxis",
            "TwistAngleCoef", "TwistMaxAngle", "KeepBoneLength", "Preset",
            // collidable object params
            "ForReplace",
            nullptr
        };
        for (int i = 0; names[i]; i++) {
            u32 h = calcCrc32(names[i], strlen(names[i]));
            dict[h] = names[i];
        }
        initialized = true;
    }
    return dict;
}

std::map<u32, std::string>& AampFile::getHashDictionary() {
    return getDictionary();
}

void AampFile::loadHashDictionary(const std::string& path) {
    initCrc32();
    auto& dict = getDictionary();
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) line[--len] = 0;
        if (len == 0 || line[0] == '#') continue;
        u32 h = calcCrc32(line, len);
        dict[h] = std::string(line, len);
    }
    fclose(f);
}

std::string AampFile::hashToName(u32 hash) {
    auto& dict = getHashDictionary();
    auto it = dict.find(hash);
    if (it != dict.end()) return it->second;
    char buf[16];
    snprintf(buf, sizeof(buf), "0x%08x", hash);
    return buf;
}

u32 AampFile::nameToHash(const std::string& name) {
    if (name.size() > 2 && name[0] == '0' && name[1] == 'x') {
        return (u32)strtoul(name.c_str(), nullptr, 16);
    }
    return calcCrc32(name.data(), name.size());
}

const char* AampFile::paramTypeName(AampParamType type) {
    switch (type) {
        case AampParamType::Bool:         return "bool";
        case AampParamType::F32:          return "f32";
        case AampParamType::Int:          return "int";
        case AampParamType::Vec2:         return "vec2";
        case AampParamType::Vec3:         return "vec3";
        case AampParamType::Vec4:         return "vec4";
        case AampParamType::Color:        return "color";
        case AampParamType::String32:     return "string32";
        case AampParamType::String64:     return "string64";
        case AampParamType::Curve1:       return "curve1";
        case AampParamType::Curve2:       return "curve2";
        case AampParamType::Curve3:       return "curve3";
        case AampParamType::Curve4:       return "curve4";
        case AampParamType::BufferInt:    return "buffer_int";
        case AampParamType::BufferF32:    return "buffer_f32";
        case AampParamType::String256:    return "string256";
        case AampParamType::Quat:         return "quat";
        case AampParamType::U32:          return "u32";
        case AampParamType::BufferU32:    return "buffer_u32";
        case AampParamType::BufferBinary: return "buffer_binary";
        case AampParamType::StringRef:    return "string_ref";
        case AampParamType::Special:      return "special";
        default:                          return "unknown";
    }
}

AampParamType AampFile::paramTypeFromName(const std::string& name) {
    if (name == "bool")          return AampParamType::Bool;
    if (name == "f32")           return AampParamType::F32;
    if (name == "int")           return AampParamType::Int;
    if (name == "vec2")          return AampParamType::Vec2;
    if (name == "vec3")          return AampParamType::Vec3;
    if (name == "vec4")          return AampParamType::Vec4;
    if (name == "color")         return AampParamType::Color;
    if (name == "string32")      return AampParamType::String32;
    if (name == "string64")      return AampParamType::String64;
    if (name == "curve1")        return AampParamType::Curve1;
    if (name == "curve2")        return AampParamType::Curve2;
    if (name == "curve3")        return AampParamType::Curve3;
    if (name == "curve4")        return AampParamType::Curve4;
    if (name == "buffer_int")    return AampParamType::BufferInt;
    if (name == "buffer_f32")    return AampParamType::BufferF32;
    if (name == "string256")     return AampParamType::String256;
    if (name == "quat")          return AampParamType::Quat;
    if (name == "u32")           return AampParamType::U32;
    if (name == "buffer_u32")    return AampParamType::BufferU32;
    if (name == "buffer_binary") return AampParamType::BufferBinary;
    if (name == "string_ref")    return AampParamType::StringRef;
    if (name == "special")       return AampParamType::Special;
    return AampParamType::Special;
}

// ─── Binary Parsing ──────────────────────────────────────────────

bool AampFile::loadFromBinary(const u8* data, size_t size) {
    if (size < 0x30) return false;
    if (memcmp(data, "AAMP", 4) != 0) return false;

    mVersion    = *(const u32*)(data + 4);
    mFlags      = *(const u32*)(data + 8);
    u32 fileSize  = *(const u32*)(data + 12);
    mPioVersion = *(const u32*)(data + 0x10);
    mOffsetToPio = *(const u32*)(data + 0x14);

    // Store PIO type string (bytes between header and root list)
    if (mOffsetToPio > 0 && 0x30 + mOffsetToPio <= size) {
        mPioTypeString.assign(data + 0x30, data + 0x30 + mOffsetToPio);
    }

    const u8* rootBase = data + 0x30 + mOffsetToPio;
    parseList(mRootList, rootBase, 0);
    return true;
}

void AampFile::parseList(AampList& list, const u8* base, size_t listOffset) const {
    const u8* p = base + listOffset;
    list.nameHash = *(const u32*)(p);
    u32 listON = *(const u32*)(p + 4);
    u32 objON  = *(const u32*)(p + 8);

    u32 numLists = listON >> 16;
    u32 listsOff = (listON & 0xFFFF) * 4;
    u32 numObjs  = objON >> 16;
    u32 objsOff  = (objON & 0xFFFF) * 4;

    for (u32 i = 0; i < numLists; i++) {
        AampList child;
        parseList(child, p, listsOff + i * 12);
        list.childLists.push_back(std::move(child));
    }

    for (u32 i = 0; i < numObjs; i++) {
        AampObject obj;
        parseObject(obj, p, objsOff + i * 8);
        list.objects.push_back(std::move(obj));
    }
}

void AampFile::parseObject(AampObject& obj, const u8* base, size_t objOffset) const {
    const u8* p = base + objOffset;
    obj.nameHash = *(const u32*)(p);
    u32 paramON = *(const u32*)(p + 4);
    u32 numParams = paramON >> 16;
    u32 paramsOff = (paramON & 0xFFFF) * 4;

    for (u32 i = 0; i < numParams; i++) {
        obj.parameters.push_back(parseParameter(p, paramsOff + i * 8));
    }
}

AampParameter AampFile::parseParameter(const u8* base, size_t paramOffset) const {
    const u8* p = base + paramOffset;
    AampParameter param;
    param.nameHash = *(const u32*)(p);
    u32 offsetAndType = *(const u32*)(p + 4);
    param.type = (AampParamType)(offsetAndType >> 24);
    u32 dataOff = (offsetAndType & 0xFFFFFF) * 4;

    const u8* dataPtr = p + dataOff;
    u8 typeId = (u8)param.type;

    switch (param.type) {
        case AampParamType::Bool:
            param.boolVal = *(const u32*)dataPtr != 0;
            break;
        case AampParamType::F32:
            param.f32Val = *(const float*)dataPtr;
            break;
        case AampParamType::Int:
            param.intVal = *(const s32*)dataPtr;
            break;
        case AampParamType::Vec2:
            memcpy(param.vec, dataPtr, 8);
            break;
        case AampParamType::Vec3:
            memcpy(param.vec, dataPtr, 12);
            break;
        case AampParamType::Vec4:
        case AampParamType::Color:
        case AampParamType::Quat:
            memcpy(param.vec, dataPtr, 16);
            break;
        case AampParamType::String32:
        case AampParamType::String64:
        case AampParamType::String256:
        case AampParamType::StringRef: {
            size_t len = strlen((const char*)dataPtr);
            param.stringVal.assign((const char*)dataPtr, len);
            break;
        }
        case AampParamType::U32:
            param.u32Val = *(const u32*)dataPtr;
            break;
        case AampParamType::Curve1:
        case AampParamType::Curve2:
        case AampParamType::Curve3:
        case AampParamType::Curve4: {
            u32 n = typeId - (u32)AampParamType::Curve1 + 1;
            u32 sz = n * 0x80;
            param.bufferData.assign(dataPtr, dataPtr + sz);
            break;
        }
        case AampParamType::BufferInt:
        case AampParamType::BufferF32:
        case AampParamType::BufferU32: {
            u32 count = *(const u32*)(dataPtr - 4);
            u32 sz = count * 4;
            param.bufferData.assign(dataPtr, dataPtr + sz);
            break;
        }
        case AampParamType::BufferBinary: {
            u32 count = *(const u32*)(dataPtr - 4);
            param.bufferData.assign(dataPtr, dataPtr + count);
            break;
        }
        default:
            break;
    }

    return param;
}

// ─── Binary Writing ──────────────────────────────────────────────

static u32 alignUp4(u32 v) { return (v + 3) & ~3u; }

static bool isStringParam(AampParamType type) {
    return type == AampParamType::String32 || type == AampParamType::String64 ||
           type == AampParamType::String256 || type == AampParamType::StringRef;
}

void AampFile::writeToBinary(std::vector<u8>& out) const {
    // Layout: Header(0x30) | PioTypeString(mOffsetToPio) | StructSection | DataSection | StringSection
    // file_size = total output size

    // Phase 1: Count lists, objects, params
    u32 numLists = 0, numObjects = 0, numParams = 0;
    std::function<void(const AampList&)> countEntries;
    countEntries = [&](const AampList& list) {
        numLists++;
        for (auto& child : list.childLists) countEntries(child);
        for (auto& obj : list.objects) {
            numObjects++;
            numParams += (u32)obj.parameters.size();
        }
    };
    countEntries(mRootList);

    // Phase 2: Build structural section - all lists first, then all objects, then all params
    // Original AAMP layout: [lists in DFS][objects in DFS][params in DFS]
    std::vector<u8> structBuf;
    struct PendingParam {
        u32 structOffset;
        const AampParameter* param;
    };
    std::vector<PendingParam> pendingParams;

    // Collect lists, objects, params in DFS order, tracking indices
    struct ListInfo {
        const AampList* list;
        u32 pos;
        u32 firstChildIdx; // index in allLists of first child (or next sibling)
        u32 firstObjIdx;   // index in allObjs of first object in this list
    };
    struct ObjInfo {
        const AampObject* obj;
        u32 pos;
        u32 firstParamIdx; // index in allParams of first param in this object
    };
    std::vector<ListInfo> allLists;
    std::vector<ObjInfo> allObjs;

    struct ParamCollect { const AampParameter* param; u32 pos; };
    std::vector<ParamCollect> allParams;

    // DFS to collect all entries in order (pre-order: list entries, then objects before children)
    std::function<void(const AampList&)> collectDFS;
    collectDFS = [&](const AampList& list) {
        u32 myIdx = (u32)allLists.size();
        u32 firstObjIdx = (u32)allObjs.size();
        allLists.push_back({&list, 0, myIdx + 1, firstObjIdx});

        // Add this list's objects BEFORE recursing into children (pre-order)
        for (auto& obj : list.objects) {
            u32 firstParamIdx = (u32)allParams.size();
            allObjs.push_back({&obj, 0, firstParamIdx});
            for (auto& p : obj.parameters)
                allParams.push_back({&p, 0});
        }

        // Recurse into children
        for (auto& child : list.childLists) collectDFS(child);
    };
    collectDFS(mRootList);

    // Assign positions: lists at 0, objects after, params after objects
    u32 listsSize = numLists * 12;
    u32 objsSize = numObjects * 8;
    u32 paramsSize = numParams * 8;
    structBuf.resize(listsSize + objsSize + paramsSize, 0);

    for (u32 i = 0; i < allLists.size(); i++)
        allLists[i].pos = i * 12;
    for (u32 i = 0; i < allObjs.size(); i++)
        allObjs[i].pos = listsSize + i * 8;
    for (u32 i = 0; i < allParams.size(); i++)
        allParams[i].pos = listsSize + objsSize + i * 8;

    // Fill list entries
    for (u32 i = 0; i < allLists.size(); i++) {
        auto& li = allLists[i];
        u32 listPos = li.pos;
        u32 nChildLists = (u32)li.list->childLists.size();
        u32 nObjs = (u32)li.list->objects.size();

        // childListsOff: first child position, or listsSize when count=0
        u32 listsOff;
        if (nChildLists > 0)
            listsOff = (allLists[li.firstChildIdx].pos - listPos) / 4;
        else
            listsOff = (listsSize - listPos) / 4;

        // objectsOff: first object position, or listsSize when count=0
        u32 objsOff;
        if (nObjs > 0)
            objsOff = (listsSize + li.firstObjIdx * 8 - listPos) / 4;
        else
            objsOff = (listsSize - listPos) / 4;

        *(u32*)(structBuf.data() + listPos + 0) = li.list->nameHash;
        *(u32*)(structBuf.data() + listPos + 4) = (nChildLists << 16) | (listsOff & 0xFFFF);
        *(u32*)(structBuf.data() + listPos + 8) = (nObjs << 16) | (objsOff & 0xFFFF);
    }

    // Fill object entries
    for (u32 i = 0; i < allObjs.size(); i++) {
        u32 objPos = allObjs[i].pos;
        auto& obj = *allObjs[i].obj;
        u32 np = (u32)obj.parameters.size();

        u32 paramAbsPos = listsSize + objsSize + allObjs[i].firstParamIdx * 8;
        u32 paramsOff = (paramAbsPos - objPos) / 4;

        *(u32*)(structBuf.data() + objPos + 0) = obj.nameHash;
        *(u32*)(structBuf.data() + objPos + 4) = (np << 16) | (paramsOff & 0xFFFF);
    }

    // Store param positions for data offset computation
    for (auto& pi : allParams)
        pendingParams.push_back({pi.pos, pi.param});

    // Phase 3: Build data section (non-strings) and string section (strings) separately
    std::vector<u8> dataSec;
    std::vector<u8> stringSec;

    // Entry dedup tracking: (data, offset)
    std::vector<std::pair<std::vector<u8>, u32>> dataEntries;
    std::vector<std::pair<std::vector<u8>, u32>> stringEntries;

    // Collect param data, building both sections
    struct ParamDataInfo {
        bool isString;
        bool isBuffer;
        u32 sectionOffset; // offset within dataSec or stringSec
        u32 dataSize;
    };
    std::vector<ParamDataInfo> paramInfos;

    for (auto& pp : pendingParams) {
        auto& param = *pp.param;
        ParamDataInfo info = {false, false, 0, 0};
        std::vector<u8> paramData;

        switch (param.type) {
            case AampParamType::Bool: {
                u32 v = param.boolVal ? 1 : 0;
                paramData.resize(4);
                memcpy(paramData.data(), &v, 4);
                break;
            }
            case AampParamType::F32:
                paramData.resize(4);
                memcpy(paramData.data(), &param.f32Val, 4);
                break;
            case AampParamType::Int:
                paramData.resize(4);
                memcpy(paramData.data(), &param.intVal, 4);
                break;
            case AampParamType::U32:
                paramData.resize(4);
                memcpy(paramData.data(), &param.u32Val, 4);
                break;
            case AampParamType::Vec2:
                paramData.resize(8);
                memcpy(paramData.data(), param.vec, 8);
                break;
            case AampParamType::Vec3:
                paramData.resize(12);
                memcpy(paramData.data(), param.vec, 12);
                break;
            case AampParamType::Vec4:
            case AampParamType::Color:
            case AampParamType::Quat:
                paramData.resize(16);
                memcpy(paramData.data(), param.vec, 16);
                break;
            case AampParamType::String32:
            case AampParamType::String64:
            case AampParamType::String256:
            case AampParamType::StringRef:
                info.isString = true;
                paramData.insert(paramData.end(), param.stringVal.begin(), param.stringVal.end());
                paramData.push_back(0);
                while (paramData.size() % 4 != 0) paramData.push_back(0);
                break;
            case AampParamType::Curve1:
            case AampParamType::Curve2:
            case AampParamType::Curve3:
            case AampParamType::Curve4:
                paramData = param.bufferData;
                break;
            case AampParamType::BufferInt:
            case AampParamType::BufferF32:
            case AampParamType::BufferU32: {
                info.isBuffer = true;
                u32 count = (u32)param.bufferData.size() / 4;
                paramData.resize(4);
                memcpy(paramData.data(), &count, 4);
                paramData.insert(paramData.end(), param.bufferData.begin(), param.bufferData.end());
                break;
            }
            case AampParamType::BufferBinary: {
                info.isBuffer = true;
                u32 count = (u32)param.bufferData.size();
                paramData.resize(4);
                memcpy(paramData.data(), &count, 4);
                paramData.insert(paramData.end(), param.bufferData.begin(), param.bufferData.end());
                while (paramData.size() % 4 != 0) paramData.push_back(0);
                break;
            }
            default: break;
        }

        info.dataSize = (u32)paramData.size();
        if (info.isString) {
            // Deduplicate: only at entry boundaries (tracked start offsets)
            bool found = false;
            for (auto& [existData, existOff] : stringEntries) {
                if (existData == paramData) {
                    info.sectionOffset = existOff;
                    found = true;
                    break;
                }
            }
            if (!found) {
                info.sectionOffset = (u32)stringSec.size();
                stringEntries.push_back({paramData, info.sectionOffset});
                stringSec.insert(stringSec.end(), paramData.begin(), paramData.end());
            }
        } else {
            // Deduplicate: only at entry boundaries (tracked start offsets)
            bool found = false;
            for (auto& [existData, existOff] : dataEntries) {
                if (existData == paramData) {
                    info.sectionOffset = existOff;
                    found = true;
                    break;
                }
            }
            if (!found) {
                info.sectionOffset = (u32)dataSec.size();
                dataEntries.push_back({paramData, info.sectionOffset});
                dataSec.insert(dataSec.end(), paramData.begin(), paramData.end());
            }
        }
        paramInfos.push_back(info);
    }

    // Phase 4: Compute relative offsets and write param entries
    u32 pioTypeSize = mOffsetToPio;
    u32 structSize = (u32)structBuf.size();
    u32 dataSecSize = (u32)dataSec.size();

    for (size_t i = 0; i < pendingParams.size(); i++) {
        auto& pp = pendingParams[i];
        auto& param = *pp.param;
        auto& info = paramInfos[i];

        // Param entry absolute position in the body (after header, relative to PIO type string start)
        u32 paramAbsInBody = pioTypeSize + pp.structOffset;

        // Data absolute position in the body
        u32 dataAbsInBody;
        if (info.isString) {
            dataAbsInBody = pioTypeSize + structSize + dataSecSize + info.sectionOffset;
        } else {
            dataAbsInBody = pioTypeSize + structSize + info.sectionOffset;
        }
        if (info.isBuffer) dataAbsInBody += 4; // skip count prefix

        u32 relOffset = (dataAbsInBody - paramAbsInBody) / 4;
        *(u32*)(structBuf.data() + pp.structOffset + 0) = param.nameHash;
        *(u32*)(structBuf.data() + pp.structOffset + 4) = ((u32)param.type << 24) | (relOffset & 0xFFFFFF);
    }

    // Phase 5: Assemble final output
    u32 bodySize = pioTypeSize + structSize + dataSecSize + (u32)stringSec.size();
    u32 totalSize = 0x30 + bodySize;
    out.resize(totalSize, 0);

    // Write header
    memcpy(out.data(), "AAMP", 4);
    *(u32*)(out.data() + 4) = mVersion;
    *(u32*)(out.data() + 8) = mFlags;
    *(u32*)(out.data() + 12) = totalSize;
    *(u32*)(out.data() + 0x10) = mPioVersion;
    *(u32*)(out.data() + 0x14) = pioTypeSize;
    *(u32*)(out.data() + 0x18) = numLists;
    *(u32*)(out.data() + 0x1C) = numObjects;
    *(u32*)(out.data() + 0x20) = numParams;
    *(u32*)(out.data() + 0x24) = dataSecSize;
    *(u32*)(out.data() + 0x28) = (u32)stringSec.size();
    *(u32*)(out.data() + 0x2C) = 0;

    // Write body
    u32 pos = 0x30;
    if (!mPioTypeString.empty()) {
        memcpy(out.data() + pos, mPioTypeString.data(), mPioTypeString.size());
    }
    pos += pioTypeSize;
    memcpy(out.data() + pos, structBuf.data(), structBuf.size());
    pos += structSize;
    if (!dataSec.empty())
        memcpy(out.data() + pos, dataSec.data(), dataSec.size());
    pos += dataSecSize;
    if (!stringSec.empty())
        memcpy(out.data() + pos, stringSec.data(), stringSec.size());
}

// ─── YAML Output ─────────────────────────────────────────────────

YAML::Node AampFile::toYaml() const {
    YAML::Node root;
    root["version"] = mVersion;
    root["flags"] = mFlags;
    root["pio_version"] = mPioVersion;
    // PIO type string (e.g. "phcl")
    if (!mPioTypeString.empty()) {
        size_t len = 0;
        while (len < mPioTypeString.size() && mPioTypeString[len] != 0) len++;
        root["pio_type"] = std::string((const char*)mPioTypeString.data(), len);
    }
    root["root"] = listToYaml(mRootList);
    return root;
}

YAML::Node AampFile::listToYaml(const AampList& list) const {
    YAML::Node node;
    node["hash"] = hashToName(list.nameHash);

    if (!list.childLists.empty()) {
        YAML::Node listsNode;
        for (auto& child : list.childLists)
            listsNode.push_back(listToYaml(child));
        node["lists"] = listsNode;
    }

    if (!list.objects.empty()) {
        YAML::Node objsNode;
        for (auto& obj : list.objects)
            objsNode.push_back(objectToYaml(obj));
        node["objects"] = objsNode;
    }

    return node;
}

YAML::Node AampFile::objectToYaml(const AampObject& obj) const {
    YAML::Node node;
    node["hash"] = hashToName(obj.nameHash);

    YAML::Node params;
    for (auto& param : obj.parameters)
        params.push_back(parameterToYaml(param));
    node["params"] = params;

    return node;
}

YAML::Node AampFile::parameterToYaml(const AampParameter& param) const {
    YAML::Node node;
    node["hash"] = hashToName(param.nameHash);
    node["type"] = paramTypeName(param.type);

    switch (param.type) {
        case AampParamType::Bool:
            node["value"] = param.boolVal;
            break;
        case AampParamType::F32:
            node["value"] = param.f32Val;
            break;
        case AampParamType::Int:
            node["value"] = param.intVal;
            break;
        case AampParamType::U32:
            node["value"] = param.u32Val;
            break;
        case AampParamType::Vec2: {
            YAML::Node v;
            v.push_back(param.vec[0]); v.push_back(param.vec[1]);
            node["value"] = v;
            break;
        }
        case AampParamType::Vec3: {
            YAML::Node v;
            v.push_back(param.vec[0]); v.push_back(param.vec[1]); v.push_back(param.vec[2]);
            node["value"] = v;
            break;
        }
        case AampParamType::Vec4:
        case AampParamType::Color:
        case AampParamType::Quat: {
            YAML::Node v;
            for (int i = 0; i < 4; i++) v.push_back(param.vec[i]);
            node["value"] = v;
            break;
        }
        case AampParamType::String32:
        case AampParamType::String64:
        case AampParamType::String256:
        case AampParamType::StringRef:
            node["value"] = param.stringVal;
            break;
        case AampParamType::Curve1:
        case AampParamType::Curve2:
        case AampParamType::Curve3:
        case AampParamType::Curve4: {
            // Curves: array of floats from the buffer
            YAML::Node v;
            const float* fp = (const float*)param.bufferData.data();
            size_t count = param.bufferData.size() / 4;
            for (size_t i = 0; i < count; i++) v.push_back(fp[i]);
            node["value"] = v;
            break;
        }
        case AampParamType::BufferInt: {
            YAML::Node v;
            const s32* ip = (const s32*)param.bufferData.data();
            size_t count = param.bufferData.size() / 4;
            for (size_t i = 0; i < count; i++) v.push_back(ip[i]);
            node["value"] = v;
            break;
        }
        case AampParamType::BufferF32: {
            YAML::Node v;
            const float* fp = (const float*)param.bufferData.data();
            size_t count = param.bufferData.size() / 4;
            for (size_t i = 0; i < count; i++) v.push_back(fp[i]);
            node["value"] = v;
            break;
        }
        case AampParamType::BufferU32: {
            YAML::Node v;
            const u32* up = (const u32*)param.bufferData.data();
            size_t count = param.bufferData.size() / 4;
            for (size_t i = 0; i < count; i++) v.push_back(up[i]);
            node["value"] = v;
            break;
        }
        case AampParamType::BufferBinary: {
            YAML::Node v;
            for (size_t i = 0; i < param.bufferData.size(); i++)
                v.push_back((int)param.bufferData[i]);
            node["value"] = v;
            break;
        }
        default:
            break;
    }

    return node;
}

// ─── YAML Input ──────────────────────────────────────────────────

bool AampFile::fromYaml(const YAML::Node& node) {
    if (!node["version"] || !node["root"]) return false;
    mVersion = node["version"].as<u32>();
    mFlags = node["flags"].as<u32>(3);
    mPioVersion = node["pio_version"].as<u32>(0);

    // Restore PIO type string
    if (node["pio_type"]) {
        std::string pioType = node["pio_type"].as<std::string>();
        mPioTypeString.resize(8, 0);
        memcpy(mPioTypeString.data(), pioType.data(), std::min(pioType.size(), (size_t)8));
        mOffsetToPio = 8;
    }

    mRootList = listFromYaml(node["root"]);
    return true;
}

AampList AampFile::listFromYaml(const YAML::Node& node) const {
    AampList list;
    list.nameHash = nameToHash(node["hash"].as<std::string>());

    if (node["lists"]) {
        for (const auto& child : node["lists"])
            list.childLists.push_back(listFromYaml(child));
    }
    if (node["objects"]) {
        for (const auto& obj : node["objects"])
            list.objects.push_back(objectFromYaml(obj));
    }
    return list;
}

AampObject AampFile::objectFromYaml(const YAML::Node& node) const {
    AampObject obj;
    obj.nameHash = nameToHash(node["hash"].as<std::string>());

    if (node["params"]) {
        for (const auto& param : node["params"])
            obj.parameters.push_back(parameterFromYaml(param));
    }
    return obj;
}

AampParameter AampFile::parameterFromYaml(const YAML::Node& node) const {
    AampParameter param;
    param.nameHash = nameToHash(node["hash"].as<std::string>());
    param.type = paramTypeFromName(node["type"].as<std::string>());

    const YAML::Node& val = node["value"];

    switch (param.type) {
        case AampParamType::Bool:
            param.boolVal = val.as<bool>();
            break;
        case AampParamType::F32:
            param.f32Val = val.as<float>();
            break;
        case AampParamType::Int:
            param.intVal = val.as<s32>();
            break;
        case AampParamType::U32:
            param.u32Val = val.as<u32>();
            break;
        case AampParamType::Vec2:
            param.vec[0] = val[0].as<float>();
            param.vec[1] = val[1].as<float>();
            break;
        case AampParamType::Vec3:
            param.vec[0] = val[0].as<float>();
            param.vec[1] = val[1].as<float>();
            param.vec[2] = val[2].as<float>();
            break;
        case AampParamType::Vec4:
        case AampParamType::Color:
        case AampParamType::Quat:
            for (int i = 0; i < 4; i++) param.vec[i] = val[i].as<float>();
            break;
        case AampParamType::String32:
        case AampParamType::String64:
        case AampParamType::String256:
        case AampParamType::StringRef:
            param.stringVal = val.as<std::string>();
            break;
        case AampParamType::Curve1:
        case AampParamType::Curve2:
        case AampParamType::Curve3:
        case AampParamType::Curve4: {
            size_t count = val.size();
            param.bufferData.resize(count * 4);
            float* fp = (float*)param.bufferData.data();
            for (size_t i = 0; i < count; i++) fp[i] = val[i].as<float>();
            break;
        }
        case AampParamType::BufferInt: {
            size_t count = val.size();
            param.bufferData.resize(count * 4);
            s32* ip = (s32*)param.bufferData.data();
            for (size_t i = 0; i < count; i++) ip[i] = val[i].as<s32>();
            break;
        }
        case AampParamType::BufferF32: {
            size_t count = val.size();
            param.bufferData.resize(count * 4);
            float* fp = (float*)param.bufferData.data();
            for (size_t i = 0; i < count; i++) fp[i] = val[i].as<float>();
            break;
        }
        case AampParamType::BufferU32: {
            size_t count = val.size();
            param.bufferData.resize(count * 4);
            u32* up = (u32*)param.bufferData.data();
            for (size_t i = 0; i < count; i++) up[i] = val[i].as<u32>();
            break;
        }
        case AampParamType::BufferBinary: {
            size_t count = val.size();
            param.bufferData.resize(count);
            for (size_t i = 0; i < count; i++) param.bufferData[i] = (u8)val[i].as<int>();
            break;
        }
        default:
            break;
    }

    return param;
}

} // namespace FiveX
