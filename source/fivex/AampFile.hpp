#pragma once

#include "types.h"
#include <vector>
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

namespace FiveX {

// AAMP parameter types (from agl::utl::ParameterBase::ParameterType)
enum class AampParamType : u8 {
    Bool        = 0,
    F32         = 1,
    Int         = 2,
    Vec2        = 3,
    Vec3        = 4,
    Vec4        = 5,
    Color       = 6,
    String32    = 7,
    String64    = 8,
    Curve1      = 9,
    Curve2      = 10,
    Curve3      = 11,
    Curve4      = 12,
    BufferInt   = 13,
    BufferF32   = 14,
    String256   = 15,
    Quat        = 16,
    U32         = 17,
    BufferU32   = 18,
    BufferBinary = 19,
    StringRef   = 20,
    Special     = 21,
};

struct AampParameter {
    u32 nameHash;
    AampParamType type;
    // Typed value storage
    bool boolVal = false;
    float f32Val = 0;
    s32 intVal = 0;
    u32 u32Val = 0;
    float vec[4] = {};
    std::string stringVal;
    std::vector<u8> bufferData;
    // Curve data: N * 0x80 bytes stored in bufferData
};

struct AampObject {
    u32 nameHash;
    std::vector<AampParameter> parameters;
};

struct AampList {
    u32 nameHash;
    std::vector<AampList> childLists;
    std::vector<AampObject> objects;
};

class AampFile {
public:
    bool loadFromBinary(const u8* data, size_t size);
    void writeToBinary(std::vector<u8>& out) const;

    YAML::Node toYaml() const;
    bool fromYaml(const YAML::Node& node);

    // Load a hash dictionary from file (one "name" per line)
    static void loadHashDictionary(const std::string& path);

    u32 mVersion = 2;
    u32 mFlags = 3; // LE + UTF8
    u32 mPioVersion = 0;
    u32 mOffsetToPio = 8;
    std::vector<u8> mPioTypeString; // e.g. "phcl\0\0\0\0"
    AampList mRootList;

private:
    void parseList(AampList& list, const u8* base, size_t listOffset) const;
    void parseObject(AampObject& obj, const u8* base, size_t objOffset) const;
    AampParameter parseParameter(const u8* base, size_t paramOffset) const;

    YAML::Node listToYaml(const AampList& list) const;
    YAML::Node objectToYaml(const AampObject& obj) const;
    YAML::Node parameterToYaml(const AampParameter& param) const;

    AampList listFromYaml(const YAML::Node& node) const;
    AampObject objectFromYaml(const YAML::Node& node) const;
    AampParameter parameterFromYaml(const YAML::Node& node) const;

    static std::string hashToName(u32 hash);
    static u32 nameToHash(const std::string& name);
    static const char* paramTypeName(AampParamType type);
    static AampParamType paramTypeFromName(const std::string& name);

    static std::map<u32, std::string>& getHashDictionary();
};

} // namespace FiveX
