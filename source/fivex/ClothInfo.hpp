#pragma once

#include "types.h"
#include "havok/hkTagfile.hpp"
#include "fivex/AampFile.hpp"
#include "phive/phive.h"
#include <vector>
#include <string>

namespace FiveX {

class ClothInfo {
public:
    ClothInfo();
    ~ClothInfo();

    void loadFromBphcl(std::vector<u8>& in);
    void serializeToBphcl(std::vector<u8>& out);

    // YAML serialization (human-readable output)
    void serializeToYaml(std::vector<u8>& outTagYaml, std::vector<u8>& outAampYaml);
    void loadFromYaml(const std::vector<u8>& tagYaml, const std::vector<u8>& aampYaml);

private:
    Havok::hkTagfile mTagFile;
    std::vector<u8> mAampSection;      // Section1 data (AAMP)
    std::vector<u8> mSection2Data;     // Section2 data (if any)
    bool mHasAamp = false;
    bool mHasSection2 = false;
};

}
