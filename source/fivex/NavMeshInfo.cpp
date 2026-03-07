#include "fivex/NavMeshInfo.hpp"
#include <cstring>
#include <cstdio>
#include <sstream>
#include <iostream>

static constexpr u32 byteswap32(u32 x) {
    return ((x << 24) & 0xFF000000) | ((x << 8) & 0x00FF0000) |
           ((x >> 8) & 0x0000FF00) | ((x >> 24) & 0x000000FF);
}

namespace FiveX {

NavMeshInfo::NavMeshInfo() {}
NavMeshInfo::~NavMeshInfo() {}

void NavMeshInfo::loadFromBphnm(std::vector<u8>& in) {
    Phive::PhiveHeader& header = *reinterpret_cast<Phive::PhiveHeader*>(in.data());

    // Section 0: TAG file
    u32 tagOffset = header.HktOffset;      // Section0 offset (always 0x30)
    u32 tagSize = header.FileSize;         // Section0 size = TAG file size

    // Section 1: ReferenceRotation data (52 bytes: u32 sectionUid + 12 floats)
    u32 extraOffset = header.TableOffset0; // Section1 offset
    u32 extraSize = header.HktSize;        // Section1 size

    // Section 2
    u32 section2Offset = header.TableOffset1;
    u32 section2Size = header.TableSize0;

    // Parse TAG file
    mTagFile.loadFromBinary(in.data() + tagOffset, tagSize);

    // Load extra section (ReferenceRotation) if present
    if (extraSize > 0 && extraOffset + extraSize <= in.size()) {
        mExtraSection.assign(in.data() + extraOffset, in.data() + extraOffset + extraSize);
        mHasExtra = true;
    } else {
        mExtraSection.clear();
        mHasExtra = false;
    }

    // Load section 2 data if present
    // Track whether section2 concept exists (s2off > 0) separately from having data
    mHasSection2 = (section2Offset > 0);
    if (section2Size > 0 && section2Offset > 0 && section2Offset + section2Size <= in.size()) {
        mSection2Data.assign(in.data() + section2Offset, in.data() + section2Offset + section2Size);
    } else {
        mSection2Data.clear();
    }
}

void NavMeshInfo::serializeToBphnm(std::vector<u8>& out) {
    // Build TAG file binary
    std::vector<u8> tagData;
    mTagFile.writeToBinary(tagData);

    // Calculate layout - all sizes computed from data
    u32 phiveHeaderSize = 0x30;
    u32 tagSizeAligned = (u32)ALIGN_UP(tagData.size(), 8);
    u32 extraSize = (u32)mExtraSection.size();
    u32 extraOffset = phiveHeaderSize + tagSizeAligned;
    u32 section2Size = (u32)mSection2Data.size();
    u32 section2Off = mHasSection2 ? (u32)ALIGN_UP(extraOffset + extraSize, 8) : 0;

    // File size only extends to s2off when section2 has actual data
    u32 totalSize = (section2Size > 0) ? (section2Off + section2Size) : (extraOffset + extraSize);

    out.resize(totalSize, 0);

    // Write Phive header
    Phive::PhiveHeader header;
    memset(&header, 0, sizeof(header));
    memcpy(header.Magic, "Phive", 6);
    header.Reserve1 = 1;
    header.BOM = 0xFEFF;
    header.MajorVersion = 1;    // bphnm version
    header.MinorVersion = 3;
    header.HktOffset = phiveHeaderSize;          // Section0 offset
    header.TableOffset0 = extraOffset;           // Section1 offset
    header.TableOffset1 = section2Off;           // Section2 offset
    header.FileSize = (u32)tagData.size();       // Section0 size = TAG size (not aligned)
    header.HktSize = extraSize;                  // Section1 size
    header.TableSize0 = section2Size;            // Section2 size

    memcpy(out.data(), &header, sizeof(header));

    // Write TAG file
    memcpy(out.data() + phiveHeaderSize, tagData.data(), tagData.size());

    // Write extra section (ReferenceRotation)
    if (mHasExtra && !mExtraSection.empty()) {
        memcpy(out.data() + extraOffset, mExtraSection.data(), mExtraSection.size());
    }

    // Write section2 data
    if (mHasSection2 && !mSection2Data.empty()) {
        memcpy(out.data() + section2Off, mSection2Data.data(), mSection2Data.size());
    }
}

void NavMeshInfo::serializeToYaml(std::vector<u8>& outYaml) {
    YAML::Node root = mTagFile.toYaml();
    root["format"] = "bphnm";

    // Decode ReferenceRotation (Section1) for human readability
    if (mHasExtra && mExtraSection.size() >= 52) {
        YAML::Node refRot(YAML::NodeType::Map);
        u32 sectionUid;
        memcpy(&sectionUid, mExtraSection.data(), 4);
        refRot["sectionUid"] = sectionUid;

        YAML::Node rotation(YAML::NodeType::Sequence);
        for (int i = 0; i < 12; i++) {
            float f;
            memcpy(&f, mExtraSection.data() + 4 + i * 4, 4);
            rotation.push_back(f);
        }
        refRot["rotation"] = rotation;
        root["referenceRotation"] = refRot;
    }

    // Decode Section2 as array of u32 entries (NavTable hashes)
    // Include empty navTable when section2 concept exists but has no data
    if (mHasSection2) {
        YAML::Node navTable(YAML::NodeType::Sequence);
        for (size_t i = 0; i + 3 < mSection2Data.size(); i += 4) {
            u32 val;
            memcpy(&val, mSection2Data.data() + i, 4);
            char hexBuf[16];
            snprintf(hexBuf, sizeof(hexBuf), "0x%08X", val);
            navTable.push_back(std::string(hexBuf));
        }
        root["navTable"] = navTable;
    }

    YAML::Emitter emit;
    emit << root;
    std::string yamlStr = emit.c_str();
    outYaml.assign(yamlStr.begin(), yamlStr.end());
}

void NavMeshInfo::loadFromYaml(const std::vector<u8>& yamlData) {
    std::string yamlStr(yamlData.begin(), yamlData.end());
    YAML::Node root = YAML::Load(yamlStr);

    mTagFile.fromYaml(root);

    // Rebuild ReferenceRotation from decoded values
    if (root["referenceRotation"]) {
        const YAML::Node& refRot = root["referenceRotation"];
        mExtraSection.resize(52, 0);
        u32 uid = refRot["sectionUid"].as<u32>();
        memcpy(mExtraSection.data(), &uid, 4);
        for (int i = 0; i < 12; i++) {
            float f = refRot["rotation"][i].as<float>();
            memcpy(mExtraSection.data() + 4 + i * 4, &f, 4);
        }
        mHasExtra = true;
    } else {
        mExtraSection.clear();
        mHasExtra = false;
    }

    // Rebuild Section2 from navTable entries
    // navTable presence indicates section2 concept exists (even if empty)
    if (root["navTable"]) {
        mHasSection2 = true;
        const YAML::Node& navTable = root["navTable"];
        mSection2Data.resize(navTable.size() * 4);
        for (size_t i = 0; i < navTable.size(); i++) {
            std::string valStr = navTable[i].as<std::string>();
            u32 val = (u32)strtoul(valStr.c_str(), nullptr, 0);
            memcpy(mSection2Data.data() + i * 4, &val, 4);
        }
    } else {
        mHasSection2 = false;
        mSection2Data.clear();
    }
}

bool NavMeshInfo::serializeToObj(std::vector<u8>& outObj) {
    const auto& items = mTagFile.mItems;
    const auto& types = mTagFile.mTypes;
    const auto& data = mTagFile.mDataSection;

    // Find the hkaiNavMesh item to get its field layout
    int navMeshItemIdx = -1;
    int faceItemIdx = -1;
    int edgeItemIdx = -1;
    int vertexItemIdx = -1;

    for (size_t i = 0; i < items.size(); i++) {
        if (items[i].typeIndex == 0) continue;
        u32 typeId = items[i].typeIndex;
        if (typeId >= types.size()) continue;
        const std::string& name = types[typeId].name;
        if (name == "hkaiNavMesh" && navMeshItemIdx < 0) {
            navMeshItemIdx = (int)i;
        } else if (name == "hkaiNavMesh::Face" && faceItemIdx < 0) {
            faceItemIdx = (int)i;
        } else if (name == "hkaiNavMesh::Edge" && edgeItemIdx < 0) {
            edgeItemIdx = (int)i;
        } else if (name == "hkVector4" && vertexItemIdx < 0) {
            // First hkVector4 array after faces/edges is the vertex array
            if (faceItemIdx >= 0 && edgeItemIdx >= 0)
                vertexItemIdx = (int)i;
        }
    }

    if (faceItemIdx < 0 || edgeItemIdx < 0 || vertexItemIdx < 0)
        return false;

    // Parse Face type to find field offsets
    const auto& faceType = types[items[faceItemIdx].typeIndex];
    u32 faceSize = faceType.size;
    u32 faceStartEdgeOff = 0, faceNumEdgesOff = 0;
    for (const auto& f : faceType.fields) {
        if (f.name == "startEdgeIndex") faceStartEdgeOff = f.offset;
        else if (f.name == "numEdges") faceNumEdgesOff = f.offset;
    }

    // Parse Edge type to find field offsets
    const auto& edgeType = types[items[edgeItemIdx].typeIndex];
    u32 edgeSize = edgeType.size;
    u32 edgeAOff = 0;
    for (const auto& f : edgeType.fields) {
        if (f.name == "a") edgeAOff = f.offset;
    }

    u32 faceCount = items[faceItemIdx].count;
    u32 faceBase = items[faceItemIdx].offset;
    u32 edgeCount = items[edgeItemIdx].count;
    u32 edgeBase = items[edgeItemIdx].offset;
    u32 vertexCount = items[vertexItemIdx].count;
    u32 vertexBase = items[vertexItemIdx].offset;

    // Build OBJ content
    std::string obj;
    obj += "# NavMesh exported from bphnm by PhiveConverter\n";
    obj += "# Vertices: " + std::to_string(vertexCount) + "\n";
    obj += "# Faces: " + std::to_string(faceCount) + "\n\n";

    // Write vertices (hkVector4 = 16 bytes per vertex: x, y, z, w as f32 LE)
    for (u32 i = 0; i < vertexCount; i++) {
        u32 off = vertexBase + i * 16;
        if (off + 16 > data.size()) break;
        float x, y, z;
        memcpy(&x, &data[off + 0], 4);
        memcpy(&y, &data[off + 4], 4);
        memcpy(&z, &data[off + 8], 4);
        char buf[128];
        snprintf(buf, sizeof(buf), "v %.9g %.9g %.9g\n", x, y, z);
        obj += buf;
    }

    obj += "\n";

    // Write faces (each face references edges by startEdgeIndex + numEdges)
    for (u32 fi = 0; fi < faceCount; fi++) {
        u32 faceOff = faceBase + fi * faceSize;
        if (faceOff + faceSize > data.size()) break;

        u32 startEdge;
        memcpy(&startEdge, &data[faceOff + faceStartEdgeOff], 4);

        u16 numEdges;
        memcpy(&numEdges, &data[faceOff + faceNumEdgesOff], 2);

        obj += "f";
        for (u16 ei = 0; ei < numEdges; ei++) {
            u32 edgeIdx = startEdge + ei;
            if (edgeIdx >= edgeCount) break;
            u32 edgeOff = edgeBase + edgeIdx * edgeSize;
            if (edgeOff + edgeSize > data.size()) break;
            u32 vertA;
            memcpy(&vertA, &data[edgeOff + edgeAOff], 4);
            obj += " " + std::to_string(vertA + 1); // OBJ is 1-indexed
        }
        obj += "\n";
    }

    outObj.assign(obj.begin(), obj.end());
    return true;
}

bool NavMeshInfo::loadFromObj(const std::vector<u8>& objData) {
    const auto& items = mTagFile.mItems;
    const auto& types = mTagFile.mTypes;
    auto& data = mTagFile.mDataSection;

    // Find the vertex item (hkVector4 array after Face and Edge items)
    int faceItemIdx = -1, edgeItemIdx = -1, vertexItemIdx = -1;
    for (size_t i = 0; i < items.size(); i++) {
        if (items[i].typeIndex == 0 || items[i].typeIndex >= types.size()) continue;
        const std::string& name = types[items[i].typeIndex].name;
        if (name == "hkaiNavMesh::Face" && faceItemIdx < 0) faceItemIdx = (int)i;
        else if (name == "hkaiNavMesh::Edge" && edgeItemIdx < 0) edgeItemIdx = (int)i;
        else if (name == "hkVector4" && vertexItemIdx < 0) {
            if (faceItemIdx >= 0 && edgeItemIdx >= 0) vertexItemIdx = (int)i;
        }
    }
    if (vertexItemIdx < 0) return false;

    u32 vertexCount = items[vertexItemIdx].count;
    u32 vertexBase = items[vertexItemIdx].offset;

    // Parse OBJ vertices
    std::vector<std::array<float, 3>> objVerts;
    std::string objStr(objData.begin(), objData.end());
    std::istringstream iss(objStr);
    std::string line;
    while (std::getline(iss, line)) {
        if (line.size() < 2 || line[0] != 'v' || line[1] != ' ') continue;
        float x, y, z;
        if (sscanf(line.c_str(), "v %f %f %f", &x, &y, &z) == 3) {
            objVerts.push_back({x, y, z});
        }
    }

    if (objVerts.size() != vertexCount) {
        std::cerr << "OBJ vertex count (" << objVerts.size()
                  << ") does not match navmesh vertex count (" << vertexCount << ")\n";
        return false;
    }

    // Patch vertex positions into the TAG data section
    for (u32 i = 0; i < vertexCount; i++) {
        u32 off = vertexBase + i * 16;
        if (off + 16 > data.size()) break;
        memcpy(&data[off + 0], &objVerts[i][0], 4);
        memcpy(&data[off + 4], &objVerts[i][1], 4);
        memcpy(&data[off + 8], &objVerts[i][2], 4);
        // w component preserved from original data
    }

    // Also update the stored base64 data_section in metadata if present
    // (Not needed for bphnm serialization which reads mDataSection directly,
    //  but ensures consistency if the data is later exported to YAML again)

    return true;
}

}
