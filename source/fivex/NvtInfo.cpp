#include "fivex/NvtInfo.hpp"
#include <cstring>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace FiveX {

NvtWaypoint NvtWaypoint::encode(float x, float y, float z) {
    NvtWaypoint wp;
    wp.xSign = std::signbit(x) ? 1 : 0;
    wp.ySign = std::signbit(y) ? 1 : 0;
    wp.zSign = std::signbit(z) ? 1 : 0;
    wp.xVal = (u16)std::round(std::fabs(x) / 256.0f * 65535.0f);
    wp.yVal = (u16)std::round(std::fabs(y) / 256.0f * 65535.0f);
    wp.zVal = (u16)std::round(std::fabs(z) / 256.0f * 65535.0f);
    return wp;
}

NvtInfo::NvtInfo() {}
NvtInfo::~NvtInfo() {}

void NvtInfo::loadFromNvt(std::vector<u8>& in) {
    if (in.size() < 8) return;

    memcpy(&mMagic, in.data(), 4);
    memcpy(&mCount, in.data() + 4, 4);

    u32 count = mCount;
    size_t offset = 8;

    // Face indices: count × u16
    mFaceIndices.resize(count);
    for (u32 i = 0; i < count; i++) {
        memcpy(&mFaceIndices[i], in.data() + offset, 2);
        offset += 2;
    }

    // Cost matrix: count × count × f32 (stride = count, no padding before matrix)
    u32 matrixEntries = count * count;
    mCostMatrix.resize(matrixEntries);
    for (u32 i = 0; i < matrixEntries; i++) {
        memcpy(&mCostMatrix[i], in.data() + offset, 4);
        offset += 4;
    }

    // Hop table: count × count × u8
    mHopTable.resize(matrixEntries);
    memcpy(mHopTable.data(), in.data() + offset, matrixEntries);
    offset += matrixEntries;

    // Path data: 9 bytes per waypoint
    size_t remaining = in.size() - offset;
    size_t waypointCount = remaining / 9;
    mWaypoints.resize(waypointCount);
    for (size_t i = 0; i < waypointCount; i++) {
        const u8* p = in.data() + offset + i * 9;
        memcpy(&mWaypoints[i].xVal, p + 0, 2);
        mWaypoints[i].xSign = p[2];
        memcpy(&mWaypoints[i].yVal, p + 3, 2);
        mWaypoints[i].ySign = p[5];
        memcpy(&mWaypoints[i].zVal, p + 6, 2);
        mWaypoints[i].zSign = p[8];
    }
}

void NvtInfo::serializeToYaml(std::vector<u8>& outYaml) {
    YAML::Node root;
    root["format"] = "nvt";
    root["magic"] = mMagic;
    root["count"] = mCount;

    // Face indices
    YAML::Node indices(YAML::NodeType::Sequence);
    for (u32 i = 0; i < mCount; i++) {
        indices.push_back((int)mFaceIndices[i]);
    }
    root["face_indices"] = indices;

    // Cost matrix as nested sequences [row][col]
    YAML::Node matrix(YAML::NodeType::Sequence);
    for (u32 i = 0; i < mCount; i++) {
        YAML::Node row(YAML::NodeType::Sequence);
        for (u32 j = 0; j < mCount; j++) {
            row.push_back(mCostMatrix[i * mCount + j]);
        }
        matrix.push_back(row);
    }
    root["cost_matrix"] = matrix;

    // Hop table as nested sequences [row][col]
    YAML::Node hops(YAML::NodeType::Sequence);
    for (u32 i = 0; i < mCount; i++) {
        YAML::Node row(YAML::NodeType::Sequence);
        for (u32 j = 0; j < mCount; j++) {
            row.push_back((int)mHopTable[i * mCount + j]);
        }
        hops.push_back(row);
    }
    root["hop_table"] = hops;

    // Path waypoints as flat list of [x, y, z]
    // Organized sequentially in row-major order matching hop_table.
    // For each (src, dst) pair where hop_table[src][dst] > 0,
    // that many consecutive waypoints define the path.
    if (!mWaypoints.empty()) {
        YAML::Node waypoints(YAML::NodeType::Sequence);
        for (size_t i = 0; i < mWaypoints.size(); i++) {
            YAML::Node pt(YAML::NodeType::Sequence);
            pt.SetStyle(YAML::EmitterStyle::Flow);
            char buf[64];
            snprintf(buf, sizeof(buf), "%.9g", mWaypoints[i].decodeX());
            pt.push_back(YAML::Load(buf));
            snprintf(buf, sizeof(buf), "%.9g", mWaypoints[i].decodeY());
            pt.push_back(YAML::Load(buf));
            snprintf(buf, sizeof(buf), "%.9g", mWaypoints[i].decodeZ());
            pt.push_back(YAML::Load(buf));
            waypoints.push_back(pt);
        }
        root["path_waypoints"] = waypoints;
    }

    YAML::Emitter emit;
    emit << root;
    std::string yamlStr = emit.c_str();
    outYaml.assign(yamlStr.begin(), yamlStr.end());
}

void NvtInfo::loadFromYaml(const std::vector<u8>& yamlData) {
    std::string yamlStr(yamlData.begin(), yamlData.end());
    YAML::Node root = YAML::Load(yamlStr);

    mMagic = root["magic"].as<u32>();
    mCount = root["count"].as<u32>();

    // Face indices
    const YAML::Node& indices = root["face_indices"];
    mFaceIndices.resize(mCount);
    for (u32 i = 0; i < mCount; i++) {
        mFaceIndices[i] = indices[i].as<u16>();
    }

    // Cost matrix
    const YAML::Node& matrix = root["cost_matrix"];
    mCostMatrix.resize(mCount * mCount);
    for (u32 i = 0; i < mCount; i++) {
        for (u32 j = 0; j < mCount; j++) {
            mCostMatrix[i * mCount + j] = matrix[i][j].as<float>();
        }
    }

    // Hop table
    const YAML::Node& hops = root["hop_table"];
    mHopTable.resize(mCount * mCount);
    for (u32 i = 0; i < mCount; i++) {
        for (u32 j = 0; j < mCount; j++) {
            mHopTable[i * mCount + j] = hops[i][j].as<int>();
        }
    }

    // Path waypoints
    mWaypoints.clear();
    if (root["path_waypoints"]) {
        const YAML::Node& wps = root["path_waypoints"];
        mWaypoints.resize(wps.size());
        for (size_t i = 0; i < wps.size(); i++) {
            float x = wps[i][0].as<float>();
            float y = wps[i][1].as<float>();
            float z = wps[i][2].as<float>();
            mWaypoints[i] = NvtWaypoint::encode(x, y, z);
        }
    }
}

void NvtInfo::serializeToNvt(std::vector<u8>& out) {
    u32 count = mCount;
    u32 matrixEntries = count * count;

    size_t totalSize = 8                         // header
                     + count * 2                 // face indices
                     + matrixEntries * 4         // cost matrix
                     + matrixEntries             // hop table
                     + mWaypoints.size() * 9;    // path waypoints

    out.resize(totalSize, 0);
    size_t offset = 0;

    // Header
    memcpy(out.data() + offset, &mMagic, 4); offset += 4;
    memcpy(out.data() + offset, &mCount, 4); offset += 4;

    // Face indices
    for (u32 i = 0; i < count; i++) {
        memcpy(out.data() + offset, &mFaceIndices[i], 2);
        offset += 2;
    }

    // Cost matrix
    for (u32 i = 0; i < matrixEntries; i++) {
        memcpy(out.data() + offset, &mCostMatrix[i], 4);
        offset += 4;
    }

    // Hop table
    memcpy(out.data() + offset, mHopTable.data(), matrixEntries);
    offset += matrixEntries;

    // Path waypoints
    for (size_t i = 0; i < mWaypoints.size(); i++) {
        u8* p = out.data() + offset + i * 9;
        memcpy(p + 0, &mWaypoints[i].xVal, 2);
        p[2] = mWaypoints[i].xSign;
        memcpy(p + 3, &mWaypoints[i].yVal, 2);
        p[5] = mWaypoints[i].ySign;
        memcpy(p + 6, &mWaypoints[i].zVal, 2);
        p[8] = mWaypoints[i].zSign;
    }
}

void NvtInfo::serializeToObj(std::vector<u8>& outObj) {
    std::ostringstream oss;
    oss << "# NVT Path Waypoints\n";
    oss << "# " << mWaypoints.size() << " waypoints\n";
    oss << "# Paths organized by hop_table: for each (src, dst) with hops > 0,\n";
    oss << "# that many consecutive vertices form the path.\n\n";

    // Write all waypoints as vertices
    for (size_t i = 0; i < mWaypoints.size(); i++) {
        char buf[128];
        snprintf(buf, sizeof(buf), "v %.9g %.9g %.9g\n",
                 mWaypoints[i].decodeX(),
                 mWaypoints[i].decodeY(),
                 mWaypoints[i].decodeZ());
        oss << buf;
    }

    // Write path polylines using the hop table
    size_t wpIdx = 1; // OBJ is 1-indexed
    for (u32 src = 0; src < mCount; src++) {
        for (u32 dst = 0; dst < mCount; dst++) {
            u8 hops = mHopTable[src * mCount + dst];
            if (hops > 0) {
                oss << "g path_" << src << "_" << dst << "\nl";
                for (u8 h = 0; h < hops; h++) {
                    oss << " " << (wpIdx + h);
                }
                oss << "\n";
                wpIdx += hops;
            }
        }
    }

    std::string s = oss.str();
    outObj.assign(s.begin(), s.end());
}

bool NvtInfo::loadFromObj(const std::vector<u8>& objData) {
    std::istringstream iss(std::string(objData.begin(), objData.end()));
    std::string line;
    std::vector<NvtWaypoint> newWaypoints;

    while (std::getline(iss, line)) {
        if (line.size() >= 2 && line[0] == 'v' && line[1] == ' ') {
            float x, y, z;
            if (sscanf(line.c_str(), "v %f %f %f", &x, &y, &z) == 3) {
                newWaypoints.push_back(NvtWaypoint::encode(x, y, z));
            }
        }
    }

    if (newWaypoints.size() != mWaypoints.size()) {
        std::cerr << "NVT OBJ vertex count mismatch: expected "
                  << mWaypoints.size() << ", got " << newWaypoints.size() << std::endl;
        return false;
    }

    mWaypoints = std::move(newWaypoints);
    return true;
}

}
