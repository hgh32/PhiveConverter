#pragma once

#include "types.h"
#include "Json.hpp"
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>
#include <yaml-cpp/yaml.h>

using json = nlohmann::json;

namespace Havok {

// Havok VLE (Variable Length Encoding) - Dlugosz format
struct VleReader {
    const u8* mData;
    size_t mPos;
    size_t mSize;

    VleReader(const u8* data, size_t size) : mData(data), mPos(0), mSize(size) {}

    bool hasMore() const { return mPos < mSize; }

    u8 readByte() {
        if (mPos >= mSize) return 0;
        return mData[mPos++];
    }

    u64 readVle() {
        u8 b0 = readByte();
        if ((b0 & 0x80) == 0) return b0;
        if ((b0 & 0xC0) == 0x80) {
            u8 b1 = readByte();
            return ((u64)(b0 & 0x3F) << 8) | b1;
        }
        if ((b0 & 0xE0) == 0xC0) {
            u8 b1 = readByte(), b2 = readByte();
            return ((u64)(b0 & 0x1F) << 16) | ((u64)b1 << 8) | b2;
        }
        if ((b0 & 0xF0) == 0xE0) {
            if ((b0 & 0xF8) == 0xE0) {
                u8 b1 = readByte(), b2 = readByte(), b3 = readByte();
                return ((u64)(b0 & 0x0F) << 24) | ((u64)b1 << 16) | ((u64)b2 << 8) | b3;
            }
            if ((b0 & 0xF8) == 0xE8) {
                u8 b1 = readByte(), b2 = readByte(), b3 = readByte(), b4 = readByte();
                return ((u64)(b0 & 0x07) << 32) | ((u64)b1 << 24) | ((u64)b2 << 16) | ((u64)b3 << 8) | b4;
            }
            if ((b0 & 0xFE) == 0xF0) {
                u64 val = 0;
                for (int i = 0; i < 7; i++) val = (val << 8) | readByte();
                return ((u64)(b0 & 0x01) << 56) | val;
            }
        }
        if (b0 == 0xF8) {
            u64 val = 0;
            for (int i = 0; i < 5; i++) val = (val << 8) | readByte();
            return val;
        }
        if (b0 == 0xF9) {
            u64 val = 0;
            for (int i = 0; i < 8; i++) val = (val << 8) | readByte();
            return val;
        }
        return 0;
    }
};

struct VleWriter {
    std::vector<u8>& mData;

    VleWriter(std::vector<u8>& data) : mData(data) {}

    void writeVle(u64 val) {
        if (val < 0x80) {
            mData.push_back((u8)val);
        } else if (val < 0x4000) {
            mData.push_back((u8)(0x80 | (val >> 8)));
            mData.push_back((u8)(val & 0xFF));
        } else if (val < 0x200000) {
            mData.push_back((u8)(0xC0 | (val >> 16)));
            mData.push_back((u8)((val >> 8) & 0xFF));
            mData.push_back((u8)(val & 0xFF));
        } else if (val < 0x10000000) {
            mData.push_back((u8)(0xE0 | (val >> 24)));
            mData.push_back((u8)((val >> 16) & 0xFF));
            mData.push_back((u8)((val >> 8) & 0xFF));
            mData.push_back((u8)(val & 0xFF));
        } else if (val < 0x800000000ULL) {
            mData.push_back((u8)(0xE8 | (val >> 32)));
            mData.push_back((u8)((val >> 24) & 0xFF));
            mData.push_back((u8)((val >> 16) & 0xFF));
            mData.push_back((u8)((val >> 8) & 0xFF));
            mData.push_back((u8)(val & 0xFF));
        } else if (val < 0x2000000000000ULL) {
            mData.push_back((u8)(0xF0 | (val >> 56)));
            for (int i = 6; i >= 0; i--)
                mData.push_back((u8)((val >> (i * 8)) & 0xFF));
        } else if (val < 0x10000000000ULL) {
            mData.push_back(0xF8);
            for (int i = 4; i >= 0; i--)
                mData.push_back((u8)((val >> (i * 8)) & 0xFF));
        } else {
            mData.push_back(0xF9);
            for (int i = 7; i >= 0; i--)
                mData.push_back((u8)((val >> (i * 8)) & 0xFF));
        }
    }
};

// Format kind constants
enum FormatKind : u32 {
    KIND_VOID    = 0,
    KIND_OPAQUE  = 1,
    KIND_BOOL    = 2,
    KIND_STRING  = 3,
    KIND_INT     = 4,
    KIND_FLOAT   = 5,
    KIND_POINTER = 6,
    KIND_RECORD  = 7,
    KIND_ARRAY   = 8,
    KIND_MASK    = 0x1F,
};

// TBDY opt bits
enum OptBits : u32 {
    OPT_FORMAT    = 1 << 0,
    OPT_SUBTYPE   = 1 << 1,
    OPT_VERSION   = 1 << 2,
    OPT_SIZE_ALIGN = 1 << 3,
    OPT_FLAGS     = 1 << 4,
    OPT_DECLS     = 1 << 5,
    OPT_INTERFACES = 1 << 6,
    OPT_ATTRIBUTE_STRING = 1 << 7,
    OPT_MUTABLE   = 1 << 8,
};

// INT format encoding
static const u32 INT_BIG_ENDIAN_BIT = 1 << 8;
static const u32 INT_SIGNED_BIT = 1 << 9;
static const u32 INT_NUM_BITS_SHIFT = 10;

struct TypeField {
    std::string name;
    u32 nameStringId = 0;
    u32 flags = 0;
    u32 offset = 0;
    u32 typeId = 0;
};

struct TypeInterface {
    u32 typeId = 0;
    u32 offset = 0;
};

struct TypeDef {
    u32 typeId = 0;
    u32 parentTypeId = 0;
    std::string name;
    u32 format = 0;
    FormatKind kind = KIND_VOID;
    u32 subtypeId = 0;
    u32 version = 0;
    u32 size = 0;
    u32 align = 1;
    u32 flags = 0;
    std::vector<TypeField> fields;
    std::vector<TypeInterface> interfaces;
    u32 optbits = 0;
};

struct TagfileItem {
    u16 typeIndex = 0;
    u8 flags = 0;
    u32 offset = 0;
    u32 count = 0;
};

struct PatchEntry {
    u32 typeIndex = 0;
    std::vector<u32> offsets;
};

class hkTagfile {
public:
    std::string mSdkVersion;
    std::vector<u8> mDataSection;
    std::vector<u8> mTypeSection;
    std::vector<TagfileItem> mItems;
    std::vector<PatchEntry> mPatches;
    std::vector<TypeDef> mTypes;

    // String tables from TYPE section
    std::vector<std::string> mTypeStrings;
    std::vector<std::string> mFieldStrings;
    std::vector<std::string> mTypeNames;

    // Structural metadata for round-tripping
    u32 mSdkvSizeRaw = 0x40000010; // Original SDKV section size+flags
    u32 mDataSizeRaw = 0;          // Original DATA section size+flags
    u32 mIndxTagOffset = 0;        // INDX start offset within TAG file (for padding)
    u32 mOrigDataPayloadSize = 0;  // Original DATA payload size (for rebuild padding)

    bool loadFromBinary(const u8* data, size_t size);
    void writeToBinary(std::vector<u8>& out) const;

    json toJson() const;
    bool fromJson(const json& j);

    YAML::Node toYaml() const;
    bool fromYaml(const YAML::Node& root);

private:
    void parseTypeSection();
    void parseTbdy(const u8* tbdyData, size_t tbdySize);
    void resolveTypeNames();

    json itemToJson(size_t itemIndex) const;
    json decodeRecord(const u8* data, const TypeDef& type) const;
    json decodeValue(const u8* data, const TypeDef& type) const;
    json decodeArray(size_t itemIndex) const;

    YAML::Node itemToYaml(size_t itemIndex) const;
    YAML::Node decodeRecordYaml(const u8* data, const TypeDef& type) const;
    YAML::Node decodeValueYaml(const u8* data, const TypeDef& type) const;

    void encodeValueYaml(const YAML::Node& node, u8* dst, const TypeDef& type) const;
    void encodeRecordYaml(const YAML::Node& node, u8* dst, const TypeDef& type) const;
    void applyYamlItemToData(const YAML::Node& itemNode, size_t itemIndex);
    void rebuildDataFromYaml(const YAML::Node& root);

    void itemFromJson(const json& j, size_t itemIndex, std::vector<u8>& dataOut) const;
    void encodeRecord(u8* data, const TypeDef& type, const json& j) const;
    void encodeValue(u8* data, const TypeDef& type, const json& j) const;

    // Helpers
    const TypeDef* getType(u32 typeId) const;
    u32 getTypeByteSize(u32 typeId) const;
    u32 getTypeAlignment(u32 typeId) const;
    bool isPointerLikeType(u32 typeId) const;
    FormatKind resolveEffectiveKind(u32 typeId) const;
    const TypeDef* resolveEffectiveType(u32 typeId) const;

    static u32 readBE32(const u8* p) {
        return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
    }
    static void writeBE32(u8* p, u32 v) {
        p[0] = (u8)(v >> 24); p[1] = (u8)(v >> 16); p[2] = (u8)(v >> 8); p[3] = (u8)v;
    }
};

} // namespace Havok
