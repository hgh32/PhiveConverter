#pragma once

#include "types.h"
#include "havok/hkTagfile.hpp"
#include "phive/phive.h"
#include <vector>
#include <string>

namespace FiveX {

class NavMeshInfo {
public:
    NavMeshInfo();
    ~NavMeshInfo();

    void loadFromBphnm(std::vector<u8>& in);
    void serializeToBphnm(std::vector<u8>& out);

    // YAML serialization
    void serializeToYaml(std::vector<u8>& outYaml);
    void loadFromYaml(const std::vector<u8>& yamlData);

    // OBJ export (navmesh visualization)
    bool serializeToObj(std::vector<u8>& outObj);

    // OBJ import (update vertex positions from edited OBJ)
    bool loadFromObj(const std::vector<u8>& objData);

private:
    Havok::hkTagfile mTagFile;
    std::vector<u8> mExtraSection;     // Section1 data (ReferenceRotation)
    std::vector<u8> mSection2Data;     // Section2 data (NavTable hashes)
    bool mHasExtra = false;
    bool mHasSection2 = false;
};

}
