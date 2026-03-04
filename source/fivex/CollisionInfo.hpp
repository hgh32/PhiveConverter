#pragma once

#include "types.h"
#include "havok/hkTypes.h"
#include "havok/hknpShape.h"
#include "phive/phive.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include "Json.hpp"

using json = nlohmann::json;

namespace FiveX {

class CollisionInfo {
public:
    struct MaterialInfo {
        int mMatId = 0;
        u64 mMatFlags = 0;
        u64 mMatColFlags = 0xFFFFFFFFFFFFFFFFULL;

        MaterialInfo() = default;
        MaterialInfo(int id, u64 flags, u64 colFlags) : mMatId(id), mMatFlags(flags), mMatColFlags(colFlags) {}
    };

    static void setConfig(json& config);

    CollisionInfo();
    ~CollisionInfo();
    void destroy();
    bool initialize(u8* objData, u64 objSize, json& matConfig);
    bool initialize(Phive::PhiveMeshShape* meshShape);
    void serializeToObj(std::vector<u8>& outObj, std::vector<u8>& outMatInfos);
    void serializeToBphsh(std::vector<u8>& out);
    void loadFromBphsh(std::vector<u8>& in);
    Phive::PhiveMeshShape* buildShape();

    std::vector<MaterialInfo> mMaterials;
    hkGeometry mGeometry;
    bool mInitialized = false;
    bool mOwnsGeometry = false;
};

}
