#pragma once

#include "types.h"
#include <vector>
#include <string>

namespace FiveX {

// A packed 3D waypoint used in nvt path data.
// Each coordinate is encoded as: (u16 / 65535.0) * 256.0 * (sign ? -1 : 1)
struct NvtWaypoint {
    u16 xVal;  u8 xSign;
    u16 yVal;  u8 ySign;
    u16 zVal;  u8 zSign;

    float decodeX() const { return (xVal / 65535.0f) * 256.0f * (xSign ? -1.0f : 1.0f); }
    float decodeY() const { return (yVal / 65535.0f) * 256.0f * (ySign ? -1.0f : 1.0f); }
    float decodeZ() const { return (zVal / 65535.0f) * 256.0f * (zSign ? -1.0f : 1.0f); }

    static NvtWaypoint encode(float x, float y, float z);
};

class NvtInfo {
public:
    NvtInfo();
    ~NvtInfo();

    void loadFromNvt(std::vector<u8>& in);
    void serializeToYaml(std::vector<u8>& outYaml);
    void loadFromYaml(const std::vector<u8>& yamlData);
    void serializeToNvt(std::vector<u8>& out);
    void serializeToObj(std::vector<u8>& outObj);
    bool loadFromObj(const std::vector<u8>& objData);

private:
    u32 mMagic = 0;
    u32 mCount = 0;
    std::vector<u16> mFaceIndices;           // count × u16 face indices
    std::vector<float> mCostMatrix;          // count × count float cost matrix
    std::vector<u8> mHopTable;               // count × count u8 hop counts
    std::vector<NvtWaypoint> mWaypoints;     // decoded path waypoints (9 bytes each)
};

}
