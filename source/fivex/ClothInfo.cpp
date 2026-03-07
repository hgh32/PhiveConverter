#include "fivex/ClothInfo.hpp"
#include <cstring>
#include <cstdio>

static constexpr u32 byteswap32(u32 x) {
    return ((x << 24) & 0xFF000000) | ((x << 8) & 0x00FF0000) |
           ((x >> 8) & 0x0000FF00) | ((x >> 24) & 0x000000FF);
}

namespace FiveX {

ClothInfo::ClothInfo() {}
ClothInfo::~ClothInfo() {}

void ClothInfo::loadFromBphcl(std::vector<u8>& in) {
    Phive::PhiveHeader& header = *reinterpret_cast<Phive::PhiveHeader*>(in.data());

    // Section 0: TAG file
    u32 tagOffset = header.HktOffset;      // Section0 offset (always 0x30)
    u32 tagSize = header.FileSize;         // Section0 size (TAG + alignment padding)

    // Section 1: AAMP data
    u32 aampOffset = header.TableOffset0;  // Section1 offset
    u32 aampSize = header.HktSize;         // Section1 size

    // Section 2
    u32 section2Offset = header.TableOffset1;
    u32 section2Size = header.TableSize0;

    // Parse TAG file (pass actual Section0 size, parser reads TAG0 internal size)
    mTagFile.loadFromBinary(in.data() + tagOffset, tagSize);

    // Load AAMP section if present
    if (aampSize > 0 && aampOffset + aampSize <= in.size()) {
        mAampSection.assign(in.data() + aampOffset, in.data() + aampOffset + aampSize);
        mHasAamp = true;
    } else {
        mAampSection.clear();
        mHasAamp = false;
    }

    // Load section 2 data if present
    if (section2Size > 0 && section2Offset > 0 && section2Offset + section2Size <= in.size()) {
        mSection2Data.assign(in.data() + section2Offset, in.data() + section2Offset + section2Size);
        mHasSection2 = true;
    } else {
        mSection2Data.clear();
        mHasSection2 = false;
    }
}

void ClothInfo::serializeToBphcl(std::vector<u8>& out) {
    // Build TAG file binary
    std::vector<u8> tagData;
    mTagFile.writeToBinary(tagData);

    // Calculate layout - all sizes computed from data
    u32 phiveHeaderSize = 0x30;
    u32 aampSize = (u32)mAampSection.size();

    // Section0 size = TAG data aligned to 16 bytes
    u32 section0Size = ((u32)tagData.size() + 15) & ~15u;

    // Section1 (AAMP) size = AAMP data aligned to 16 bytes
    u32 section1Size = (aampSize > 0) ? ((aampSize + 15) & ~15u) : 0;

    u32 aampOffset = phiveHeaderSize + section0Size;
    u32 section2Off = aampOffset + section1Size;
    u32 section2Size = (u32)mSection2Data.size();

    u32 totalSize = section2Off + section2Size;

    out.resize(totalSize, 0);

    // Write Phive header
    Phive::PhiveHeader header;
    memset(&header, 0, sizeof(header));
    memcpy(header.Magic, "Phive", 6);
    header.Reserve1 = 1;
    header.BOM = 0xFEFF;
    header.MajorVersion = 3;    // bphcl version
    header.MinorVersion = 3;
    header.HktOffset = phiveHeaderSize;          // Section0 offset
    header.TableOffset0 = aampOffset;            // Section1 offset
    header.TableOffset1 = section2Off;           // Section2 offset
    header.FileSize = section0Size;              // Section0 size
    header.HktSize = section1Size;               // Section1 size
    header.TableSize0 = section2Size;            // Section2 size

    memcpy(out.data(), &header, sizeof(header));

    // Write TAG file
    memcpy(out.data() + phiveHeaderSize, tagData.data(), tagData.size());

    // Write AAMP
    if (mHasAamp && !mAampSection.empty()) {
        memcpy(out.data() + aampOffset, mAampSection.data(), mAampSection.size());
    }

    // Write section2 data
    if (mHasSection2 && !mSection2Data.empty()) {
        memcpy(out.data() + section2Off, mSection2Data.data(), mSection2Data.size());
    }
}

void ClothInfo::serializeToYaml(std::vector<u8>& outTagYaml, std::vector<u8>& outAampYaml) {
    // Main TAG YAML with decoded items
    YAML::Node root = mTagFile.toYaml();
    root["format"] = "bphcl";

    YAML::Emitter tagEmit;
    tagEmit << root;
    std::string tagStr = tagEmit.c_str();
    outTagYaml.assign(tagStr.begin(), tagStr.end());

    // AAMP YAML - human-readable decoded parameters
    outAampYaml.clear();
    if (mHasAamp && !mAampSection.empty()) {
        AampFile aamp;
        if (aamp.loadFromBinary(mAampSection.data(), mAampSection.size())) {
            YAML::Node aampRoot = aamp.toYaml();
            YAML::Emitter aampEmit;
            aampEmit << aampRoot;
            std::string aampStr = aampEmit.c_str();
            outAampYaml.assign(aampStr.begin(), aampStr.end());
        }
    }
}

void ClothInfo::loadFromYaml(const std::vector<u8>& tagYaml, const std::vector<u8>& aampYaml) {
    std::string tagStr(tagYaml.begin(), tagYaml.end());
    YAML::Node root = YAML::Load(tagStr);

    mTagFile.fromYaml(root);

    // Rebuild AAMP from decoded YAML
    if (!aampYaml.empty()) {
        std::string aampStr(aampYaml.begin(), aampYaml.end());
        YAML::Node aampRoot = YAML::Load(aampStr);
        AampFile aamp;
        if (aamp.fromYaml(aampRoot)) {
            aamp.writeToBinary(mAampSection);
            mHasAamp = true;
        } else {
            mAampSection.clear();
            mHasAamp = false;
        }
    } else {
        mAampSection.clear();
        mHasAamp = false;
    }

    // Section2 is empty in all bphcl files
    mSection2Data.clear();
    mHasSection2 = false;
}

}
