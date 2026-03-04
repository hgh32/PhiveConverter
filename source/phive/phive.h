#pragma once

#include "types.h"
#include "havok/hknpShape.h"

namespace Phive {

struct PhiveHeader {
    char Magic[6];
    uint16_t Reserve1 = 0;
    uint16_t BOM = 0;
    uint8_t MajorVersion = 0;
    uint8_t MinorVersion = 0;
    uint32_t HktOffset = 0;
    uint32_t TableOffset0 = 0;
    uint32_t TableOffset1 = 0;
    uint32_t FileSize = 0;
    uint32_t HktSize = 0;
    uint32_t TableSize0 = 0;
    uint32_t TableSize1 = 0;
    uint32_t Reserve2 = 0;
    uint32_t Reserve3 = 0;
};

struct PhiveShapeMaterialData {
    int mMatId = 0;
    int _4 = 0;
    u64 mFlags = 0;

    PhiveShapeMaterialData() = default;
    PhiveShapeMaterialData(int matId, int pad, u64 flags) : mMatId(matId), _4(pad), mFlags(flags) {}
};

struct PhiveMeshShape {
    int mMaterialNum = 0;
    PhiveShapeMaterialData* mMaterialArray = nullptr;
    int mMatColFlagsNum = 0;
    u64* mMatColFlags = nullptr;
    hknpMeshShape* mShape = nullptr;

    ~PhiveMeshShape() {
        delete[] mMaterialArray;
        delete[] mMatColFlags;
    }
};

}
