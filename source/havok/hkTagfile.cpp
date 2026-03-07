#include "havok/hkTagfile.hpp"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <set>
#include <map>

namespace {

// Base64 encoding/decoding for binary metadata storage
static const char b64chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64Encode(const std::vector<u8>& data) {
    std::string out;
    out.reserve((data.size() + 2) / 3 * 4);
    for (size_t i = 0; i < data.size(); i += 3) {
        u32 n = (u32)data[i] << 16;
        if (i + 1 < data.size()) n |= (u32)data[i+1] << 8;
        if (i + 2 < data.size()) n |= data[i+2];
        out += b64chars[(n >> 18) & 0x3F];
        out += b64chars[(n >> 12) & 0x3F];
        out += (i + 1 < data.size()) ? b64chars[(n >> 6) & 0x3F] : '=';
        out += (i + 2 < data.size()) ? b64chars[n & 0x3F] : '=';
    }
    return out;
}

std::vector<u8> base64Decode(const std::string& str) {
    static int lut[256] = {-1};
    if (lut[0] == -1) {
        for (int i = 0; i < 256; i++) lut[i] = -1;
        for (int i = 0; i < 64; i++) lut[(u8)b64chars[i]] = i;
    }
    std::vector<u8> out;
    out.reserve(str.size() * 3 / 4);
    u32 buf = 0;
    int bits = 0;
    for (char c : str) {
        if (lut[(u8)c] < 0) continue;
        buf = (buf << 6) | lut[(u8)c];
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back((u8)(buf >> bits));
        }
    }
    return out;
}

} // anonymous namespace

namespace Havok {

static constexpr u32 byteswap32(u32 x) {
    return ((x << 24) & 0xFF000000) | ((x << 8) & 0x00FF0000) |
           ((x >> 8) & 0x0000FF00) | ((x >> 24) & 0x000000FF);
}

// NaN-preserving float serialization: YAML's .nan loses the specific NaN payload,
// so we store NaN as a tagged hex string to preserve the exact bit pattern.
YAML::Node floatToYaml(float f) {
    if (std::isnan(f)) {
        u32 bits;
        memcpy(&bits, &f, 4);
        char buf[16];
        snprintf(buf, sizeof(buf), "0x%08X", bits);
        YAML::Node n;
        n.SetTag("!nan");
        n = std::string(buf);
        return n;
    }
    return YAML::Node(f);
}

float yamlToFloat(const YAML::Node& node) {
    if (node.Tag() == "!nan" || node.Tag() == "nan") {
        u32 bits = (u32)std::stoul(node.as<std::string>(), nullptr, 16);
        float f;
        memcpy(&f, &bits, 4);
        return f;
    }
    return node.as<float>();
}

// ============================================================================
// Binary Loading
// ============================================================================

bool hkTagfile::loadFromBinary(const u8* data, size_t size) {
    if (size < 8) return false;

    // TAG0 header
    u32 tag0Size = readBE32(data) & 0x3FFFFFFF;
    if (memcmp(data + 4, "TAG0", 4) != 0) return false;

    const u8* pos = data + 8;
    const u8* tagEnd = data + tag0Size;

    // Parse sections within TAG0
    const u8* dataPayload = nullptr;
    size_t dataPayloadSize = 0;
    const u8* typePayload = nullptr;
    size_t typePayloadSize = 0;
    const u8* indxPayload = nullptr;
    size_t indxPayloadSize = 0;

    while (pos < tagEnd) {
        u32 secSizeRaw = readBE32(pos);
        u32 secSize = secSizeRaw & 0x3FFFFFFF;
        if (secSize < 8) {
            // Skip zero padding (occurs between TYPE and INDX sections)
            pos += 4;
            continue;
        }
        char secMagic[5] = {};
        memcpy(secMagic, pos + 4, 4);

        if (strcmp(secMagic, "SDKV") == 0) {
            mSdkvSizeRaw = secSizeRaw;
            mSdkVersion = std::string((const char*)pos + 8, secSize - 8);
            // Trim any trailing nulls
            while (!mSdkVersion.empty() && mSdkVersion.back() == '\0')
                mSdkVersion.pop_back();
        } else if (strcmp(secMagic, "DATA") == 0) {
            mDataSizeRaw = secSizeRaw;
            dataPayload = pos + 8;
            dataPayloadSize = secSize - 8;
        } else if (strcmp(secMagic, "TYPE") == 0) {
            typePayload = pos;
            typePayloadSize = secSize;
        } else if (strcmp(secMagic, "INDX") == 0) {
            mIndxTagOffset = (u32)(pos - data);
            indxPayload = pos + 8;
            indxPayloadSize = secSize - 8;
        }

        pos += secSize;
    }

    if (!dataPayload || !typePayload || !indxPayload) return false;

    // Store DATA section raw bytes
    mDataSection.assign(dataPayload, dataPayload + dataPayloadSize);
    mOrigDataPayloadSize = (u32)dataPayloadSize;

    // Store TYPE section verbatim (including its own size+magic header)
    mTypeSection.assign(typePayload, typePayload + typePayloadSize);

    // Parse TYPE section for type definitions
    parseTypeSection();

    // Parse INDX section (ITEM + PTCH)
    const u8* indxPos = indxPayload;
    const u8* indxEnd = indxPayload + indxPayloadSize;

    while (indxPos < indxEnd) {
        u32 subSecSize = readBE32(indxPos) & 0x3FFFFFFF;
        if (subSecSize == 0) break;
        char subMagic[5] = {};
        memcpy(subMagic, indxPos + 4, 4);

        if (strcmp(subMagic, "ITEM") == 0) {
            const u8* itemData = indxPos + 8;
            size_t itemDataSize = subSecSize - 8;
            size_t itemCount = itemDataSize / 12;
            mItems.resize(itemCount);

            for (size_t i = 0; i < itemCount; i++) {
                const u8* entry = itemData + i * 12;
                mItems[i].typeIndex = *(u16*)(entry);
                // entry[2] is padding
                mItems[i].flags = entry[3];
                mItems[i].offset = *(u32*)(entry + 4);
                mItems[i].count = *(u32*)(entry + 8);
            }
        } else if (strcmp(subMagic, "PTCH") == 0) {
            const u8* ptchData = indxPos + 8;
            size_t ptchDataSize = subSecSize - 8;
            const u8* p = ptchData;
            const u8* ptchEnd = ptchData + ptchDataSize;

            while (p < ptchEnd) {
                PatchEntry patch;
                patch.typeIndex = *(u32*)p;
                u32 cnt = *(u32*)(p + 4);
                p += 8;
                patch.offsets.resize(cnt);
                for (u32 j = 0; j < cnt; j++) {
                    patch.offsets[j] = *(u32*)p;
                    p += 4;
                }
                mPatches.push_back(std::move(patch));
            }
        }

        indxPos += subSecSize;
    }

    return true;
}

// ============================================================================
// TYPE Section Parsing
// ============================================================================

void hkTagfile::parseTypeSection() {
    if (mTypeSection.size() < 8) return;

    const u8* typeStart = mTypeSection.data();
    u32 typeSize = readBE32(typeStart) & 0x3FFFFFFF;
    const u8* pos = typeStart + 8; // skip TYPE size+magic
    const u8* typeEnd = typeStart + typeSize;

    const u8* tst1Data = nullptr; size_t tst1Size = 0;
    const u8* tna1Data = nullptr; size_t tna1Size = 0;
    const u8* fst1Data = nullptr; size_t fst1Size = 0;
    const u8* tbdyData = nullptr; size_t tbdySize = 0;

    while (pos < typeEnd) {
        u32 subSize = readBE32(pos) & 0x3FFFFFFF;
        if (subSize == 0) break; // Prevent infinite loop
        char subMagic[5] = {};
        memcpy(subMagic, pos + 4, 4);

        if (strcmp(subMagic, "TST1") == 0) { tst1Data = pos + 8; tst1Size = subSize - 8; }
        else if (strcmp(subMagic, "TNA1") == 0) { tna1Data = pos + 8; tna1Size = subSize - 8; }
        else if (strcmp(subMagic, "FST1") == 0) { fst1Data = pos + 8; fst1Size = subSize - 8; }
        else if (strcmp(subMagic, "TBDY") == 0) { tbdyData = pos + 8; tbdySize = subSize - 8; }

        pos += subSize;
    }

    // Parse string tables
    if (tst1Data) {
        mTypeStrings.clear();
        const char* s = (const char*)tst1Data;
        const char* end = s + tst1Size;
        while (s < end) {
            size_t len = strnlen(s, end - s);
            mTypeStrings.push_back(std::string(s, len));
            s += len + 1;
        }
    }

    if (fst1Data) {
        mFieldStrings.clear();
        const char* s = (const char*)fst1Data;
        const char* end = s + fst1Size;
        while (s < end) {
            size_t len = strnlen(s, end - s);
            mFieldStrings.push_back(std::string(s, len));
            s += len + 1;
        }
    }

    // Parse TNA1 (type name association: sequential entries, NOT VLE)
    // Format: count(u8), then (count-1) entries of: index(u8) + templateCount(u8) + templates
    if (tna1Data && tna1Size > 0) {
        mTypeNames.clear();
        const u8* p = tna1Data;
        const u8* end = tna1Data + tna1Size;

        u8 count = *p++;
        u32 numEntries = count - 1;
        mTypeNames.resize(numEntries + 1); // typeIds start at 1

        for (u32 i = 0; i < numEntries && p < end; i++) {
            u8 strIdx = *p++;
            u8 templateCount = *p++;

            u32 typeId = i + 1; // TNA1 entries correspond to TBDY typeIds 1, 2, 3, ...
            if (strIdx < mTypeStrings.size())
                mTypeNames[typeId] = mTypeStrings[strIdx];

            // Skip template entries (2 bytes each: index + value)
            for (u8 t = 0; t < templateCount && p + 1 < end; t++) {
                p += 2;
            }
        }
    }

    // Parse TBDY
    if (tbdyData) {
        parseTbdy(tbdyData, tbdySize);
        resolveTypeNames();
    }
}

void hkTagfile::parseTbdy(const u8* tbdyData, size_t tbdySize) {
    VleReader reader(tbdyData, tbdySize);
    mTypes.clear();

    while (reader.mPos < tbdySize) {
        TypeDef type;
        type.typeId = (u32)reader.readVle();
        type.parentTypeId = (u32)reader.readVle();
        type.optbits = (u32)reader.readVle();

        if (type.optbits & OPT_FORMAT) {
            type.format = (u32)reader.readVle();
            type.kind = (FormatKind)(type.format & KIND_MASK);
        }
        if (type.optbits & OPT_SUBTYPE)
            type.subtypeId = (u32)reader.readVle();
        if (type.optbits & OPT_VERSION)
            type.version = (u32)reader.readVle();
        if (type.optbits & OPT_SIZE_ALIGN) {
            type.size = (u32)reader.readVle();
            type.align = (u32)reader.readVle();
        }
        if (type.optbits & OPT_FLAGS)
            type.flags = (u32)reader.readVle();
        if (type.optbits & OPT_DECLS) {
            u64 encoded = reader.readVle();
            u32 numFields = (u32)(encoded & 0xFFFF);
            u32 numProperties = (u32)(encoded >> 16);
            (void)numProperties;
            type.fields.resize(numFields);
            for (u32 i = 0; i < numFields; i++) {
                type.fields[i].nameStringId = (u32)reader.readVle();
                type.fields[i].flags = (u32)reader.readVle();
                type.fields[i].offset = (u32)reader.readVle();
                type.fields[i].typeId = (u32)reader.readVle();
            }
        }
        if (type.optbits & OPT_INTERFACES) {
            u32 numIfaces = (u32)reader.readVle();
            type.interfaces.resize(numIfaces);
            for (u32 i = 0; i < numIfaces; i++) {
                type.interfaces[i].typeId = (u32)reader.readVle();
                type.interfaces[i].offset = (u32)reader.readVle();
            }
        }
        if (type.optbits & OPT_ATTRIBUTE_STRING) {
            reader.readVle(); // skip attribute string id
        }
        if (type.optbits & OPT_MUTABLE) {
            reader.readVle(); // skip mutable info
        }

        // Ensure types vector is large enough
        if (type.typeId >= mTypes.size())
            mTypes.resize(type.typeId + 1);
        mTypes[type.typeId] = std::move(type);
    }
}

void hkTagfile::resolveTypeNames() {
    for (auto& type : mTypes) {
        if (type.typeId == 0) continue;
        // Set name from TNA1 mapping
        if (type.typeId < mTypeNames.size() && !mTypeNames[type.typeId].empty())
            type.name = mTypeNames[type.typeId];
        // Set field names from FST1
        for (auto& field : type.fields) {
            if (field.nameStringId < mFieldStrings.size())
                field.name = mFieldStrings[field.nameStringId];
        }
    }
}

// ============================================================================
// Type Helpers
// ============================================================================

const TypeDef* hkTagfile::getType(u32 typeId) const {
    if (typeId < mTypes.size() && mTypes[typeId].typeId == typeId)
        return &mTypes[typeId];
    return nullptr;
}

u32 hkTagfile::getTypeByteSize(u32 typeId) const {
    const TypeDef* type = getType(typeId);
    if (!type) return 0;
    if (type->size > 0) return type->size;
    // Inherit size from parent chain
    if (type->parentTypeId != 0)
        return getTypeByteSize(type->parentTypeId);
    return 0;
}

u32 hkTagfile::getTypeAlignment(u32 typeId) const {
    const TypeDef* type = getType(typeId);
    if (!type) return 1;
    if (type->align > 1 || (type->optbits & OPT_SIZE_ALIGN)) return type->align > 0 ? type->align : 1;
    // Inherit alignment from parent chain
    if (type->parentTypeId != 0)
        return getTypeAlignment(type->parentTypeId);
    return 1;
}

bool hkTagfile::isPointerLikeType(u32 typeId) const {
    const TypeDef* type = getType(typeId);
    if (!type) return false;
    if (type->kind == KIND_POINTER || type->kind == KIND_STRING || type->kind == KIND_ARRAY)
        return true;
    // Check parent chain
    if (type->kind == KIND_VOID && type->parentTypeId != 0)
        return isPointerLikeType(type->parentTypeId);
    return false;
}

FormatKind hkTagfile::resolveEffectiveKind(u32 typeId) const {
    const TypeDef* type = getType(typeId);
    if (!type) return KIND_VOID;
    if (type->kind != KIND_VOID) return type->kind;
    if (type->parentTypeId != 0) return resolveEffectiveKind(type->parentTypeId);
    return KIND_VOID;
}

const TypeDef* hkTagfile::resolveEffectiveType(u32 typeId) const {
    const TypeDef* type = getType(typeId);
    if (!type) return nullptr;
    if (type->kind != KIND_VOID || type->parentTypeId == 0) return type;
    return resolveEffectiveType(type->parentTypeId);
}

// ============================================================================
// JSON Serialization
// ============================================================================

json hkTagfile::decodeValue(const u8* data, const TypeDef& type) const {
    switch (type.kind) {
        case KIND_INT: {
            u32 numBits = type.format >> INT_NUM_BITS_SHIFT;
            bool isSigned = (type.format & INT_SIGNED_BIT) != 0;
            if (numBits <= 8) {
                return isSigned ? json((s8)data[0]) : json(data[0]);
            } else if (numBits <= 16) {
                u16 v = *(const u16*)data;
                return isSigned ? json((s16)v) : json(v);
            } else if (numBits <= 32) {
                u32 v = *(const u32*)data;
                return isSigned ? json((s32)v) : json(v);
            } else {
                u64 v = *(const u64*)data;
                return isSigned ? json((s64)v) : json(v);
            }
        }
        case KIND_FLOAT: {
            if (type.size == 4) {
                float v;
                memcpy(&v, data, 4);
                return json(v);
            } else if (type.size == 8) {
                double v;
                memcpy(&v, data, 8);
                return json(v);
            } else if (type.size == 2) {
                // Half float - store as raw u16 for now
                u16 v = *(const u16*)data;
                return json(v);
            }
            return json(nullptr);
        }
        case KIND_BOOL: {
            if (type.size == 1) return json(data[0] != 0);
            if (type.size == 4) return json(*(const u32*)data != 0);
            return json(data[0] != 0);
        }
        case KIND_POINTER:
        case KIND_STRING: {
            // Pointer fields store item index as u64
            u64 itemRef = *(const u64*)data;
            if (itemRef == 0) return json(nullptr);
            json ref;
            ref["$ref"] = itemRef;
            return ref;
        }
        default:
            break;
    }

    // Opaque/void or unknown: return hex
    if (type.size > 0) {
        std::string hex;
        hex.reserve(type.size * 2);
        for (u32 i = 0; i < type.size; i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x", data[i]);
            hex += buf;
        }
        return json(hex);
    }
    return json(nullptr);
}

json hkTagfile::decodeRecord(const u8* data, const TypeDef& type) const {
    json obj = json::object();

    for (const auto& field : type.fields) {
        const TypeDef* fieldType = getType(field.typeId);
        if (!fieldType) continue;

        const u8* fieldData = data + field.offset;

        // Resolve effective kind through parent chain
        FormatKind fKind = resolveEffectiveKind(field.typeId);
        const TypeDef* fEffective = resolveEffectiveType(field.typeId);
        if (!fEffective) fEffective = fieldType;

        if (fKind == KIND_RECORD) {
            // Find type with actual fields
            const TypeDef* rt = fEffective;
            while (rt && rt->fields.empty() && rt->parentTypeId != 0)
                rt = getType(rt->parentTypeId);
            if (!rt) rt = fEffective;
            obj[field.name] = decodeRecord(fieldData, *rt);
        } else if (fKind == KIND_ARRAY) {
            json arrObj = json::object();
            u64 itemRef = *(const u64*)fieldData;
            arrObj["$ref"] = itemRef;
            arrObj["m_size"] = *(const s32*)(fieldData + 8);
            arrObj["m_capacityAndFlags"] = *(const s32*)(fieldData + 12);
            obj[field.name] = arrObj;
        } else {
            obj[field.name] = decodeValue(fieldData, *fEffective);
        }
    }

    // Store any bytes not covered by fields as gaps
    u32 typeSize = getTypeByteSize(type.typeId);
    if (typeSize > 0) {
        std::vector<bool> covered(typeSize, false);
        for (const auto& field : type.fields) {
            u32 fieldSize = getTypeByteSize(field.typeId);
            for (u32 i = field.offset; i < field.offset + fieldSize && i < typeSize; i++)
                covered[i] = true;
        }

        bool hasGap = false;
        for (u32 i = 0; i < typeSize; i++) {
            if (!covered[i] && data[i] != 0) { hasGap = true; break; }
        }

        if (hasGap) {
            std::string gapHex;
            gapHex.reserve(typeSize * 2);
            for (u32 i = 0; i < typeSize; i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02x", data[i]);
                gapHex += buf;
            }
            obj["_raw"] = gapHex;
        }
    }

    return obj;
}

json hkTagfile::itemToJson(size_t itemIndex) const {
    if (itemIndex == 0 || itemIndex >= mItems.size()) return json(nullptr);

    const auto& item = mItems[itemIndex];
    const TypeDef* type = getType(item.typeIndex);

    json result = json::object();
    if (type) {
        result["_type"] = type->name.empty() ?
            ("type_" + std::to_string(item.typeIndex)) : type->name;
    } else {
        result["_type"] = "unknown_type_" + std::to_string(item.typeIndex);
    }
    result["_typeIndex"] = item.typeIndex;
    result["_flags"] = item.flags;
    result["_offset"] = item.offset;
    result["_count"] = item.count;

    u32 elementSize = type ? getTypeByteSize(item.typeIndex) : 0;

    // Store raw hex for every item to guarantee perfect round-tripping
    if (item.count > 0 && elementSize > 0) {
        size_t totalSize = (size_t)item.count * elementSize;
        if (item.offset + totalSize <= mDataSection.size()) {
            const u8* rawData = mDataSection.data() + item.offset;
            std::string hex;
            hex.reserve(totalSize * 2);
            for (size_t i = 0; i < totalSize; i++) {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02x", rawData[i]);
                hex += buf;
            }
            result["_hex"] = hex;
        }
    }

    // Resolve the effective type kind through parent chain (only if type is known)
    if (!type) return result;

    FormatKind effectiveKind = resolveEffectiveKind(item.typeIndex);
    const TypeDef* effectiveType = resolveEffectiveType(item.typeIndex);
    if (!effectiveType) effectiveType = type;

    if (effectiveKind == KIND_RECORD && item.count > 0 && elementSize > 0) {
        // Find the type with actual fields (may need to walk parent chain)
        const TypeDef* recordType = effectiveType;
        while (recordType && recordType->fields.empty() && recordType->parentTypeId != 0) {
            const TypeDef* parent = getType(recordType->parentTypeId);
            if (parent) recordType = parent;
            else break;
        }

        if (item.count == 1 && item.flags == 0x10) {
            const u8* objData = mDataSection.data() + item.offset;
            result["_fields"] = decodeRecord(objData, *recordType);
        } else {
            json elements = json::array();
            for (u32 i = 0; i < item.count; i++) {
                const u8* objData = mDataSection.data() + item.offset + i * elementSize;
                elements.push_back(decodeRecord(objData, *recordType));
            }
            result["_elements"] = elements;
        }
    } else if (effectiveKind == KIND_INT && item.count > 0) {
        u32 numBits = effectiveType->format >> INT_NUM_BITS_SHIFT;
        if (numBits <= 8) {
            const u8* strData = mDataSection.data() + item.offset;
            bool isString = true;
            for (u32 i = 0; i < item.count; i++) {
                u8 c = strData[i];
                if (c != 0 && (c < 0x20 || c > 0x7E)) { isString = false; break; }
            }
            if (isString && item.count > 0) {
                size_t len = strnlen((const char*)strData, item.count);
                result["_string"] = std::string((const char*)strData, len);
            }
        } else {
            json arr = json::array();
            const u8* arrData = mDataSection.data() + item.offset;
            for (u32 i = 0; i < item.count; i++) {
                const u8* elem = arrData + i * elementSize;
                arr.push_back(decodeValue(elem, *effectiveType));
            }
            result["_elements"] = arr;
        }
    } else if (effectiveKind == KIND_FLOAT && item.count > 0) {
        json arr = json::array();
        const u8* arrData = mDataSection.data() + item.offset;
        for (u32 i = 0; i < item.count; i++) {
            const u8* elem = arrData + i * elementSize;
            arr.push_back(decodeValue(elem, *effectiveType));
        }
        result["_elements"] = arr;
    } else if (effectiveKind == KIND_BOOL && item.count > 0) {
        json arr = json::array();
        const u8* arrData = mDataSection.data() + item.offset;
        for (u32 i = 0; i < item.count; i++) {
            const u8* elem = arrData + i * elementSize;
            arr.push_back(decodeValue(elem, *effectiveType));
        }
        result["_elements"] = arr;
    }

    return result;
}

json hkTagfile::toJson() const {
    json root = json::object();
    root["sdk_version"] = mSdkVersion;
    root["data_size"] = (u64)mDataSection.size();
    root["sdkv_size_raw"] = mSdkvSizeRaw;
    root["data_size_raw"] = mDataSizeRaw;
    root["indx_tag_offset"] = mIndxTagOffset;

    // Store TYPE section as hex for verbatim round-trip
    {
        std::string typeHex;
        typeHex.reserve(mTypeSection.size() * 2);
        for (size_t i = 0; i < mTypeSection.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x", mTypeSection[i]);
            typeHex += buf;
        }
        root["type_section"] = typeHex;
    }

    // Store ENTIRE DATA section as hex for guaranteed perfect round-trip
    {
        std::string dataHex;
        dataHex.reserve(mDataSection.size() * 2);
        for (size_t i = 0; i < mDataSection.size(); i++) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02x", mDataSection[i]);
            dataHex += buf;
        }
        root["data_section"] = dataHex;
    }

    // Items (decoded for human readability)
    json items = json::array();
    items.push_back(nullptr); // item 0 is always null
    for (size_t i = 1; i < mItems.size(); i++) {
        items.push_back(itemToJson(i));
    }
    root["items"] = items;

    // Patches (for reference)
    json patches = json::array();
    for (const auto& patch : mPatches) {
        json p = json::object();
        const TypeDef* patchType = getType(patch.typeIndex);
        if (patchType && !patchType->name.empty())
            p["type"] = patchType->name;
        p["typeIndex"] = patch.typeIndex;
        p["offsets"] = patch.offsets;
        patches.push_back(p);
    }
    root["patches"] = patches;

    return root;
}

// ============================================================================
// JSON Deserialization
// ============================================================================

void hkTagfile::encodeValue(u8* data, const TypeDef& type, const json& j) const {
    switch (type.kind) {
        case KIND_INT: {
            u32 numBits = type.format >> INT_NUM_BITS_SHIFT;
            bool isSigned = (type.format & INT_SIGNED_BIT) != 0;
            if (numBits <= 8) {
                if (isSigned) data[0] = (u8)(s8)j.get<int>();
                else data[0] = (u8)j.get<int>();
            } else if (numBits <= 16) {
                u16 v;
                if (isSigned) v = (u16)(s16)j.get<int>();
                else v = (u16)j.get<int>();
                memcpy(data, &v, 2);
            } else if (numBits <= 32) {
                if (isSigned) {
                    s32 v = j.get<s32>();
                    memcpy(data, &v, 4);
                } else {
                    u32 v = j.get<u32>();
                    memcpy(data, &v, 4);
                }
            } else {
                if (isSigned) {
                    s64 v = j.get<s64>();
                    memcpy(data, &v, 8);
                } else {
                    u64 v = j.get<u64>();
                    memcpy(data, &v, 8);
                }
            }
            break;
        }
        case KIND_FLOAT: {
            if (type.size == 4) {
                float v = j.get<float>();
                memcpy(data, &v, 4);
            } else if (type.size == 8) {
                double v = j.get<double>();
                memcpy(data, &v, 8);
            } else if (type.size == 2) {
                u16 v = j.get<u16>();
                memcpy(data, &v, 2);
            }
            break;
        }
        case KIND_BOOL: {
            bool v = j.get<bool>();
            if (type.size == 1) data[0] = v ? 1 : 0;
            else if (type.size == 4) { u32 bv = v ? 1 : 0; memcpy(data, &bv, 4); }
            break;
        }
        case KIND_POINTER:
        case KIND_STRING: {
            u64 itemRef = 0;
            if (!j.is_null() && j.is_object() && j.contains("$ref"))
                itemRef = j["$ref"].get<u64>();
            memcpy(data, &itemRef, 8);
            break;
        }
        default: break;
    }
}

void hkTagfile::encodeRecord(u8* data, const TypeDef& type, const json& j) const {
    // If there's a _raw field, start from that
    if (j.contains("_raw")) {
        std::string hex = j["_raw"].get<std::string>();
        u32 typeSize = getTypeByteSize(type.typeId);
        for (size_t i = 0; i + 1 < hex.size() && i / 2 < typeSize; i += 2) {
            char byte[3] = {hex[i], hex[i+1], 0};
            data[i/2] = (u8)strtoul(byte, nullptr, 16);
        }
    }

    for (const auto& field : type.fields) {
        if (!j.contains(field.name)) continue;
        const json& fieldJson = j[field.name];
        const TypeDef* fieldType = getType(field.typeId);
        if (!fieldType) continue;

        u8* fieldData = data + field.offset;

        FormatKind fKind = resolveEffectiveKind(field.typeId);
        const TypeDef* fEffective = resolveEffectiveType(field.typeId);
        if (!fEffective) fEffective = fieldType;

        if (fKind == KIND_RECORD) {
            const TypeDef* rt = fEffective;
            while (rt && rt->fields.empty() && rt->parentTypeId != 0)
                rt = getType(rt->parentTypeId);
            if (!rt) rt = fEffective;
            encodeRecord(fieldData, *rt, fieldJson);
        } else if (fKind == KIND_ARRAY) {
            u64 itemRef = 0;
            s32 mSize = 0, mCapFlags = 0;
            if (fieldJson.is_object()) {
                if (fieldJson.contains("$ref"))
                    itemRef = fieldJson["$ref"].get<u64>();
                if (fieldJson.contains("m_size"))
                    mSize = fieldJson["m_size"].get<s32>();
                if (fieldJson.contains("m_capacityAndFlags"))
                    mCapFlags = fieldJson["m_capacityAndFlags"].get<s32>();
            }
            memcpy(fieldData, &itemRef, 8);
            memcpy(fieldData + 8, &mSize, 4);
            memcpy(fieldData + 12, &mCapFlags, 4);
        } else {
            encodeValue(fieldData, *fEffective, fieldJson);
        }
    }
}

bool hkTagfile::fromJson(const json& j) {
    mSdkVersion = j["sdk_version"].get<std::string>();

    // Restore structural metadata
    mSdkvSizeRaw = j.value("sdkv_size_raw", (u32)0x40000010);
    mDataSizeRaw = j.value("data_size_raw", (u32)0);
    mIndxTagOffset = j.value("indx_tag_offset", (u32)0);

    // Restore TYPE section from hex
    std::string typeHex = j["type_section"].get<std::string>();
    mTypeSection.resize(typeHex.size() / 2);
    for (size_t i = 0; i + 1 < typeHex.size(); i += 2) {
        char byte[3] = {typeHex[i], typeHex[i+1], 0};
        mTypeSection[i/2] = (u8)strtoul(byte, nullptr, 16);
    }

    // Re-parse TYPE section
    parseTypeSection();

    // Restore DATA section from hex (primary path for perfect round-trip)
    if (j.contains("data_section")) {
        std::string dataHex = j["data_section"].get<std::string>();
        mDataSection.resize(dataHex.size() / 2);
        for (size_t i = 0; i + 1 < dataHex.size(); i += 2) {
            char byte[3] = {dataHex[i], dataHex[i+1], 0};
            mDataSection[i/2] = (u8)strtoul(byte, nullptr, 16);
        }
    } else {
        // Fallback: reconstruct DATA from per-item _hex fields
        u64 dataSize = 0;
        if (j.contains("data_size"))
            dataSize = j["data_size"].get<u64>();
        mDataSection.resize(dataSize, 0);
    }

    // Parse items from JSON (metadata for ITEM table)
    const json& jsonItems = j["items"];
    mItems.resize(jsonItems.size());
    mItems[0] = {}; // null item

    for (size_t i = 1; i < jsonItems.size(); i++) {
        const json& itemJson = jsonItems[i];
        if (itemJson.is_null()) continue;

        mItems[i].typeIndex = itemJson["_typeIndex"].get<u16>();
        mItems[i].flags = itemJson["_flags"].get<u8>();

        if (itemJson.contains("_offset"))
            mItems[i].offset = itemJson["_offset"].get<u32>();
        else
            mItems[i].offset = 0;

        if (itemJson.contains("_count")) {
            mItems[i].count = itemJson["_count"].get<u32>();
        } else if (itemJson.contains("_elements")) {
            mItems[i].count = (u32)itemJson["_elements"].size();
        } else if (itemJson.contains("_string")) {
            std::string str = itemJson["_string"].get<std::string>();
            mItems[i].count = (u32)(str.size() + 1);
        } else if (itemJson.contains("_hex")) {
            u32 elementSize = getTypeByteSize(mItems[i].typeIndex);
            std::string hex = itemJson["_hex"].get<std::string>();
            u32 totalBytes = (u32)(hex.size() / 2);
            mItems[i].count = elementSize > 0 ? totalBytes / elementSize : totalBytes;
        } else if (itemJson.contains("_fields")) {
            mItems[i].count = 1;
        } else {
            mItems[i].count = 1;
        }
    }

    // If no data_section was available, try reconstructing from per-item _hex
    if (!j.contains("data_section")) {
        for (size_t i = 1; i < jsonItems.size(); i++) {
            const json& itemJson = jsonItems[i];
            if (itemJson.is_null()) continue;
            if (!itemJson.contains("_hex")) continue;
            if (mItems[i].offset >= mDataSection.size()) continue;

            std::string hex = itemJson["_hex"].get<std::string>();
            u8* itemData = mDataSection.data() + mItems[i].offset;
            for (size_t h = 0; h + 1 < hex.size(); h += 2) {
                char byte[3] = {hex[h], hex[h+1], 0};
                if (mItems[i].offset + h/2 < mDataSection.size())
                    itemData[h/2] = (u8)strtoul(byte, nullptr, 16);
            }
        }
    }

    // Reconstruct patches from JSON
    mPatches.clear();
    if (j.contains("patches")) {
        for (const auto& pj : j["patches"]) {
            PatchEntry patch;
            patch.typeIndex = pj["typeIndex"].get<u32>();
            for (const auto& off : pj["offsets"])
                patch.offsets.push_back(off.get<u32>());
            mPatches.push_back(std::move(patch));
        }
    }

    return true;
}

// ============================================================================
// YAML - Value Decoding
// ============================================================================

YAML::Node hkTagfile::decodeValueYaml(const u8* data, const TypeDef& type) const {
    YAML::Node node;
    switch (type.kind) {
        case KIND_INT: {
            u32 numBits = type.format >> INT_NUM_BITS_SHIFT;
            bool isSigned = (type.format & INT_SIGNED_BIT) != 0;
            if (numBits <= 8) {
                node = isSigned ? (int)(s8)data[0] : (int)data[0];
            } else if (numBits <= 16) {
                u16 raw; memcpy(&raw, data, 2);
                node = isSigned ? (int)(s16)raw : (int)raw;
            } else if (numBits <= 32) {
                if (isSigned) { s32 v; memcpy(&v, data, 4); node = v; }
                else { u32 v; memcpy(&v, data, 4); node = (int64_t)v; }
            } else {
                if (isSigned) { s64 v; memcpy(&v, data, 8); node = v; }
                else { u64 v; memcpy(&v, data, 8); node = v; }
            }
            break;
        }
        case KIND_FLOAT: {
            if (type.size == 4) {
                float v; memcpy(&v, data, 4);
                node = floatToYaml(v);
            } else if (type.size == 8) {
                double v; memcpy(&v, data, 8);
                node = v;
            } else if (type.size == 2) {
                u16 raw; memcpy(&raw, data, 2);
                // half-float → store as raw int for lossless round-trip
                node = (int)raw;
                node.SetTag("!half");
            }
            break;
        }
        case KIND_BOOL: {
            if (type.size == 1) node = (data[0] != 0);
            else if (type.size == 4) { u32 v; memcpy(&v, data, 4); node = (v != 0); }
            else node = (data[0] != 0);
            break;
        }
        case KIND_POINTER:
        case KIND_STRING: {
            // Pointers resolve to item references
            u64 ptrVal; memcpy(&ptrVal, data, 8);
            if (ptrVal == 0) {
                node = YAML::Node(YAML::NodeType::Null);
            } else {
                // Find the item at this offset (only valid if fits in u32)
                if (ptrVal <= UINT32_MAX) {
                    for (size_t i = 1; i < mItems.size(); i++) {
                        if (mItems[i].offset == (u32)ptrVal) {
                            // If it's a string item, inline the string value
                            if (type.kind == KIND_STRING) {
                                const u8* strData = mDataSection.data() + mItems[i].offset;
                                node = std::string((const char*)strData);
                            } else {
                                node = (int64_t)i;
                                node.SetTag("!ref");
                            }
                            break;
                        }
                    }
                }
                if (!node.IsDefined() || (node.IsNull() && ptrVal != 0)) {
                    node = (int64_t)ptrVal;
                    node.SetTag("!ptr");
                }
            }
            break;
        }
        default:
            node = YAML::Node(YAML::NodeType::Null);
            break;
    }
    return node;
}

// ============================================================================
// YAML - Record Decoding
// ============================================================================

YAML::Node hkTagfile::decodeRecordYaml(const u8* data, const TypeDef& type) const {
    // Special handling: hkVector4/hkVector4f types should display as 4 floats
    {
        bool isVec4 = false;
        const TypeDef* check = &type;
        while (check) {
            if (check->name == "hkVector4" || check->name == "hkVector4f") {
                isVec4 = true;
                break;
            }
            if (check->parentTypeId != 0)
                check = getType(check->parentTypeId);
            else
                check = nullptr;
        }
        if (isVec4) {
            YAML::Node vec(YAML::NodeType::Sequence);
            for (int i = 0; i < 4; i++) {
                float f;
                memcpy(&f, data + i * 4, 4);
                vec.push_back(floatToYaml(f));
            }
            return vec;
        }
    }

    YAML::Node node(YAML::NodeType::Map);

    // Follow parent chain to get all fields
    const TypeDef* cur = &type;
    while (cur) {
        for (const auto& field : cur->fields) {
            const u8* fieldData = data + field.offset;
            const TypeDef* fieldType = getType(field.typeId);
            if (!fieldType) continue;

            FormatKind fKind = resolveEffectiveKind(field.typeId);
            const TypeDef* fEffective = resolveEffectiveType(field.typeId);
            if (!fEffective) fEffective = fieldType;

            if (fKind == KIND_RECORD) {
                // Inline record - walk parent chain to find fields
                const TypeDef* rt = fEffective;
                while (rt && rt->fields.empty() && rt->parentTypeId != 0)
                    rt = getType(rt->parentTypeId);
                if (!rt) rt = fEffective;
                node[field.name] = decodeRecordYaml(fieldData, *rt);
            } else if (fKind == KIND_ARRAY) {
                // Distinguish hkArray (with m_data/m_size/m_capacityAndFlags fields)
                // from fixed-size inline arrays like hkVector4f (no fields, just subtype)
                const TypeDef* arrType = fieldType;
                bool isHkArray = false;
                const TypeDef* checkArr = arrType;
                while (checkArr) {
                    if (!checkArr->fields.empty()) {
                        for (const auto& af : checkArr->fields) {
                            if (af.name == "m_data") { isHkArray = true; break; }
                        }
                        break;
                    }
                    if (checkArr->parentTypeId != 0)
                        checkArr = getType(checkArr->parentTypeId);
                    else
                        checkArr = nullptr;
                }

                if (!isHkArray && fEffective && fEffective->subtypeId != 0 && fEffective->size > 0) {
                    // Fixed-size inline array (e.g., hkVector4f = 4 floats)
                    const TypeDef* elemType = getType(fEffective->subtypeId);
                    u32 elemSize = elemType ? elemType->size : 0;
                    if (elemType && elemSize > 0) {
                        u32 elemCount = fEffective->size / elemSize;
                        YAML::Node vec(YAML::NodeType::Sequence);
                        for (u32 vi = 0; vi < elemCount; vi++) {
                            if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                                float f;
                                memcpy(&f, fieldData + vi * 4, 4);
                                vec.push_back(floatToYaml(f));
                            } else if (elemType->kind == KIND_INT && elemSize == 4) {
                                s32 val;
                                memcpy(&val, fieldData + vi * 4, 4);
                                vec.push_back(val);
                            } else {
                                vec.push_back((int)fieldData[vi * elemSize]);
                            }
                        }
                        node[field.name] = vec;
                    } else {
                        // Fallback: store as raw values
                        node[field.name] = decodeValueYaml(fieldData, *fEffective);
                    }
                } else {
                    // Standard hkArray: ptr(8) + size(4) + capAndFlags(4) = 16 bytes
                    u64 ptr; memcpy(&ptr, fieldData, 8);
                    s32 mSize; memcpy(&mSize, fieldData + 8, 4);
                    s32 mCapFlags; memcpy(&mCapFlags, fieldData + 12, 4);

                    YAML::Node arrNode(YAML::NodeType::Map);
                    if (ptr == 0) {
                        arrNode["$ref"] = YAML::Node(YAML::NodeType::Null);
                    } else {
                        // Find item at this offset
                        for (size_t i = 1; i < mItems.size(); i++) {
                            if (mItems[i].offset == (u32)ptr) {
                                arrNode["$ref"] = (int64_t)i;
                                break;
                            }
                        }
                    }
                    arrNode["m_size"] = mSize;
                    arrNode["m_capacityAndFlags"] = mCapFlags;
                    node[field.name] = arrNode;
                }
            } else {
                node[field.name] = decodeValueYaml(fieldData, *fEffective);
            }
        }
        if (cur->parentTypeId != 0) cur = getType(cur->parentTypeId);
        else cur = nullptr;
    }

    return node;
}

// ============================================================================
// YAML - Item Decoding
// ============================================================================

YAML::Node hkTagfile::itemToYaml(size_t itemIndex) const {
    if (itemIndex >= mItems.size()) return YAML::Node(YAML::NodeType::Null);
    const auto& item = mItems[itemIndex];

    if (item.offset >= mDataSection.size() && item.offset != 0)
        return YAML::Node(YAML::NodeType::Null);

    const TypeDef* type = getType(item.typeIndex);

    YAML::Node node(YAML::NodeType::Map);
    node["_typeIndex"] = item.typeIndex;
    node["_flags"] = (int)item.flags;

    if (type && !type->name.empty())
        node["_typeName"] = type->name;

    FormatKind kind = resolveEffectiveKind(item.typeIndex);
    const TypeDef* effective = resolveEffectiveType(item.typeIndex);
    if (!effective) effective = type;

    const u8* itemData = mDataSection.data() + item.offset;
    u32 elementSize = type ? getTypeByteSize(item.typeIndex) : 0;

    // tT is always char (1 byte) regardless of type table inheritance
    bool isCharType = (type && (type->name == "tT" || type->name == "char"));
    if (isCharType) elementSize = 1;

    u32 totalBytes = elementSize > 0 ? elementSize * item.count : 0;

    // String items (including tT char arrays)
    if (kind == KIND_STRING || isCharType) {
        u32 strLen = item.count > 0 ? item.count - 1 : 0;
        if (item.offset + strLen <= mDataSection.size())
            node["_string"] = std::string((const char*)itemData, strLen);
        return node;
    }

    // Items with known type - decode fields
    // Records and opaque structs (KIND_VOID with size > 0 treated as potential records)
    if (type && effective && (kind == KIND_RECORD || (kind == KIND_VOID && elementSize > 0))) {
        const TypeDef* rt = effective;
        while (rt && rt->fields.empty() && rt->parentTypeId != 0)
            rt = getType(rt->parentTypeId);
        if (!rt || rt->fields.empty()) {
            // No field info - store as byte array
            if (totalBytes > 0 && item.offset + totalBytes <= mDataSection.size()) {
                YAML::Node bytes(YAML::NodeType::Sequence);
                for (u32 b = 0; b < totalBytes; b++)
                    bytes.push_back((int)itemData[b]);
                node["_bytes"] = bytes;
            }
            return node;
        }

        if (item.flags == 0x10) {
            // Single record
            node["_fields"] = decodeRecordYaml(itemData, *rt);
        } else if (item.flags == 0x20 && item.count > 0) {
            // Array of records
            YAML::Node elements(YAML::NodeType::Sequence);
            for (u32 e = 0; e < item.count; e++) {
                const u8* eData = itemData + e * elementSize;
                if (item.offset + (e + 1) * elementSize <= mDataSection.size())
                    elements.push_back(decodeRecordYaml(eData, *rt));
            }
            node["_elements"] = elements;
        }
        return node;
    }

    // Items with known non-record types (int, float, bool, pointer arrays)
    if (type && effective && (kind == KIND_INT || kind == KIND_FLOAT || kind == KIND_BOOL || kind == KIND_POINTER)) {
        // Check for char/byte arrays that are actually strings
        if (kind == KIND_INT && elementSize == 1 && item.flags == 0x20 && item.count > 0) {
            // Check if this looks like a null-terminated string
            u32 strLen = 0;
            bool isAscii = true;
            for (u32 i = 0; i < item.count && item.offset + i < mDataSection.size(); i++) {
                u8 c = itemData[i];
                if (c == 0) { strLen = i; break; }
                if (c < 0x20 || c > 0x7E) { isAscii = false; break; }
                strLen = i + 1;
            }
            if (isAscii && strLen > 0) {
                node["_string"] = std::string((const char*)itemData, strLen);
                return node;
            }
        }

        if (item.count == 1 || item.flags == 0x10) {
            node["_value"] = decodeValueYaml(itemData, *effective);
        } else {
            YAML::Node elements(YAML::NodeType::Sequence);
            for (u32 e = 0; e < item.count; e++) {
                const u8* eData = itemData + e * elementSize;
                if (item.offset + (e + 1) * elementSize <= mDataSection.size())
                    elements.push_back(decodeValueYaml(eData, *effective));
            }
            node["_elements"] = elements;
        }
        return node;
    }

    // Fixed-size inline array types (e.g., hkVector4f = 4 floats inline)
    if (type && effective && kind == KIND_ARRAY && effective->subtypeId != 0 && effective->size > 0) {
        const TypeDef* elemType = getType(effective->subtypeId);
        u32 elemSize = elemType ? elemType->size : 0;
        if (elemType && elemSize > 0) {
            // Use the item's own type size (elementSize) when it's smaller than the
            // ancestor array's size - the type overrides the array's inline extent
            u32 vecSize = (elementSize > 0 && elementSize < effective->size) ? elementSize : effective->size;
            u32 elemsPerVec = vecSize / elemSize;
            if (item.count == 1 || item.flags == 0x10) {
                // Single vector
                YAML::Node vec(YAML::NodeType::Sequence);
                for (u32 vi = 0; vi < elemsPerVec; vi++) {
                    if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                        float f; memcpy(&f, itemData + vi * 4, 4);
                        vec.push_back(floatToYaml(f));
                    } else {
                        s32 val; memcpy(&val, itemData + vi * elemSize, std::min(elemSize, 4u));
                        vec.push_back(val);
                    }
                }
                node["_elements"] = vec;
            } else if (item.count > 0) {
                // Array of vectors
                YAML::Node elements(YAML::NodeType::Sequence);
                for (u32 e = 0; e < item.count; e++) {
                    const u8* eData = itemData + e * vecSize;
                    if (item.offset + (e + 1) * vecSize > mDataSection.size()) break;
                    YAML::Node vec(YAML::NodeType::Sequence);
                    for (u32 vi = 0; vi < elemsPerVec; vi++) {
                        if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                            float f; memcpy(&f, eData + vi * 4, 4);
                            vec.push_back(floatToYaml(f));
                        } else {
                            s32 val; memcpy(&val, eData + vi * elemSize, std::min(elemSize, 4u));
                            vec.push_back(val);
                        }
                    }
                    elements.push_back(vec);
                }
                node["_elements"] = elements;
            }
            return node;
        }
    }

    // Unknown type - store as byte array (decimal, not hex)
    // Only use item.count as byte count if elementSize > 0 (avoids reading random bytes
    // for zero-size reference anchor items)
    if (totalBytes == 0 && elementSize > 0) totalBytes = item.count;
    if (totalBytes > 0 && item.offset + totalBytes <= mDataSection.size()) {
        YAML::Node bytes(YAML::NodeType::Sequence);
        for (u32 b = 0; b < totalBytes; b++)
            bytes.push_back((int)itemData[b]);
        node["_bytes"] = bytes;
    }

    return node;
}

// ============================================================================
// YAML Serialization
// ============================================================================

YAML::Node hkTagfile::toYaml() const {
    YAML::Node root(YAML::NodeType::Map);
    root["sdk_version"] = mSdkVersion;

    // Items (decoded for human readability)
    YAML::Node items(YAML::NodeType::Sequence);
    items.push_back(YAML::Node(YAML::NodeType::Null)); // item 0 is always null
    for (size_t i = 1; i < mItems.size(); i++) {
        items.push_back(itemToYaml(i));
    }
    root["items"] = items;

    // Patches (for reference, human-readable)
    YAML::Node patches(YAML::NodeType::Sequence);
    for (const auto& patch : mPatches) {
        YAML::Node p(YAML::NodeType::Map);
        const TypeDef* patchType = getType(patch.typeIndex);
        if (patchType && !patchType->name.empty())
            p["type"] = patchType->name;
        p["typeIndex"] = patch.typeIndex;
        YAML::Node offsets(YAML::NodeType::Sequence);
        for (u32 off : patch.offsets) offsets.push_back(off);
        p["offsets"] = offsets;
        patches.push_back(p);
    }
    root["patches"] = patches;

    // ---- Metadata section (for byte-perfect round-trip) ----
    YAML::Node metadata(YAML::NodeType::Map);
    metadata["sdkv_size_raw"] = mSdkvSizeRaw;
    metadata["data_size_raw"] = mDataSizeRaw;
    metadata["indx_tag_offset"] = mIndxTagOffset;
    metadata["data_size"] = (u64)mDataSection.size();
    metadata["orig_data_payload_size"] = mOrigDataPayloadSize;

    // Type section as base64
    metadata["type_section"] = base64Encode(mTypeSection);

    // Data section as base64
    metadata["data_section"] = base64Encode(mDataSection);

    // Item layout (offsets/counts for reconstruction)
    YAML::Node itemLayout(YAML::NodeType::Sequence);
    for (size_t i = 0; i < mItems.size(); i++) {
        YAML::Node il(YAML::NodeType::Map);
        il["typeIndex"] = mItems[i].typeIndex;
        il["flags"] = (int)mItems[i].flags;
        il["offset"] = mItems[i].offset;
        il["count"] = mItems[i].count;
        itemLayout.push_back(il);
    }
    metadata["item_layout"] = itemLayout;

    root["_metadata"] = metadata;

    return root;
}

// ============================================================================
// YAML - Value Encoding (inverse of decodeValueYaml)
// ============================================================================

void hkTagfile::encodeValueYaml(const YAML::Node& node, u8* dst, const TypeDef& type) const {
    switch (type.kind) {
        case KIND_INT: {
            u32 numBits = type.format >> INT_NUM_BITS_SHIFT;
            bool isSigned = (type.format & INT_SIGNED_BIT) != 0;
            if (numBits <= 8) {
                if (isSigned) dst[0] = (u8)(s8)node.as<int>();
                else dst[0] = (u8)node.as<int>();
            } else if (numBits <= 16) {
                u16 v;
                if (isSigned) v = (u16)(s16)node.as<int>();
                else v = (u16)node.as<int>();
                memcpy(dst, &v, 2);
            } else if (numBits <= 32) {
                if (isSigned) { s32 v = node.as<s32>(); memcpy(dst, &v, 4); }
                else { u32 v = (u32)node.as<int64_t>(); memcpy(dst, &v, 4); }
            } else {
                if (isSigned) { s64 v = node.as<s64>(); memcpy(dst, &v, 8); }
                else { u64 v = node.as<u64>(); memcpy(dst, &v, 8); }
            }
            break;
        }
        case KIND_FLOAT: {
            if (type.size == 4) {
                float v = yamlToFloat(node);
                memcpy(dst, &v, 4);
            } else if (type.size == 8) {
                double v = node.as<double>();
                memcpy(dst, &v, 8);
            } else if (type.size == 2) {
                // Half-float stored as raw u16 with !half tag
                u16 v = (u16)node.as<int>();
                memcpy(dst, &v, 2);
            }
            break;
        }
        case KIND_BOOL: {
            bool v = node.as<bool>();
            if (type.size == 1) dst[0] = v ? 1 : 0;
            else if (type.size == 4) { u32 bv = v ? 1 : 0; memcpy(dst, &bv, 4); }
            break;
        }
        case KIND_POINTER:
        case KIND_STRING: {
            // Encode pointer values from YAML tagged nodes
            if (node.Tag() == "!ref") {
                size_t refIdx = node.as<size_t>();
                u64 ptrVal = 0;
                if (refIdx < mItems.size())
                    ptrVal = mItems[refIdx].offset;
                memcpy(dst, &ptrVal, 8);
            } else if (node.Tag() == "!ptr") {
                u64 ptrVal = (u64)node.as<int64_t>();
                memcpy(dst, &ptrVal, 8);
            }
            // Null and untagged: keep original bytes from data copy
            break;
        }
        default: break;
    }
}

void hkTagfile::encodeRecordYaml(const YAML::Node& node, u8* dst, const TypeDef& type) const {
    // hkVector4/hkVector4f: sequence of 4 floats
    {
        bool isVec4 = false;
        const TypeDef* check = &type;
        while (check) {
            if (check->name == "hkVector4" || check->name == "hkVector4f") {
                isVec4 = true;
                break;
            }
            if (check->parentTypeId != 0)
                check = getType(check->parentTypeId);
            else
                check = nullptr;
        }
        if (isVec4 && node.IsSequence() && node.size() == 4) {
            for (int i = 0; i < 4; i++) {
                float f = yamlToFloat(node[i]);
                memcpy(dst + i * 4, &f, 4);
            }
            return;
        }
    }

    if (!node.IsMap()) return;

    // Follow parent chain to encode all fields
    const TypeDef* cur = &type;
    while (cur) {
        for (const auto& field : cur->fields) {
            if (!node[field.name]) continue;
            const YAML::Node& fieldNode = node[field.name];
            const TypeDef* fieldType = getType(field.typeId);
            if (!fieldType) continue;

            u8* fieldData = dst + field.offset;

            FormatKind fKind = resolveEffectiveKind(field.typeId);
            const TypeDef* fEffective = resolveEffectiveType(field.typeId);
            if (!fEffective) fEffective = fieldType;

            if (fKind == KIND_RECORD) {
                const TypeDef* rt = fEffective;
                while (rt && rt->fields.empty() && rt->parentTypeId != 0)
                    rt = getType(rt->parentTypeId);
                if (!rt) rt = fEffective;
                encodeRecordYaml(fieldNode, fieldData, *rt);
            } else if (fKind == KIND_ARRAY) {
                // Distinguish hkArray from fixed-size inline arrays
                const TypeDef* arrType = fieldType;
                bool isHkArray = false;
                const TypeDef* checkArr = arrType;
                while (checkArr) {
                    if (!checkArr->fields.empty()) {
                        for (const auto& af : checkArr->fields) {
                            if (af.name == "m_data") { isHkArray = true; break; }
                        }
                        break;
                    }
                    if (checkArr->parentTypeId != 0)
                        checkArr = getType(checkArr->parentTypeId);
                    else
                        checkArr = nullptr;
                }

                if (!isHkArray && fEffective && fEffective->subtypeId != 0 && fEffective->size > 0) {
                    // Fixed-size inline array (e.g., 4 floats for hkVector4f)
                    const TypeDef* elemType = getType(fEffective->subtypeId);
                    u32 elemSize = elemType ? elemType->size : 0;
                    if (elemType && elemSize > 0 && fieldNode.IsSequence()) {
                        u32 elemCount = fEffective->size / elemSize;
                        for (u32 vi = 0; vi < elemCount && vi < fieldNode.size(); vi++) {
                            if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                                float f = yamlToFloat(fieldNode[vi]);
                                memcpy(fieldData + vi * 4, &f, 4);
                            } else if (elemType->kind == KIND_INT && elemSize == 4) {
                                s32 val = fieldNode[vi].as<s32>();
                                memcpy(fieldData + vi * 4, &val, 4);
                            } else {
                                fieldData[vi * elemSize] = (u8)fieldNode[vi].as<int>();
                            }
                        }
                    }
                } else if (fieldNode.IsMap()) {
                    // Standard hkArray: keep pointer, write m_size and m_capacityAndFlags
                    // Pointer ($ref) is NOT modified - it stays from metadata
                    if (fieldNode["m_size"]) {
                        s32 mSize = fieldNode["m_size"].as<s32>();
                        memcpy(fieldData + 8, &mSize, 4);
                    }
                    if (fieldNode["m_capacityAndFlags"]) {
                        s32 mCapFlags = fieldNode["m_capacityAndFlags"].as<s32>();
                        memcpy(fieldData + 12, &mCapFlags, 4);
                    }
                    // Handle $ref (pointer to element data item)
                    if (fieldNode["$ref"] && !fieldNode["$ref"].IsNull()) {
                        size_t refIdx = fieldNode["$ref"].as<size_t>();
                        if (refIdx < mItems.size()) {
                            u64 ptrVal = mItems[refIdx].offset;
                            memcpy(fieldData, &ptrVal, 8);
                        }
                    }
                }
            } else {
                encodeValueYaml(fieldNode, fieldData, *fEffective);
            }
        }
        if (cur->parentTypeId != 0) cur = getType(cur->parentTypeId);
        else cur = nullptr;
    }
}

void hkTagfile::applyYamlItemToData(const YAML::Node& itemNode, size_t itemIndex) {
    if (itemNode.IsNull() || itemIndex >= mItems.size()) return;

    const auto& item = mItems[itemIndex];
    if (item.offset >= mDataSection.size()) return;

    const TypeDef* type = getType(item.typeIndex);
    FormatKind kind = resolveEffectiveKind(item.typeIndex);
    const TypeDef* effective = resolveEffectiveType(item.typeIndex);
    if (!effective) effective = type;

    // Ensure data section is large enough for this item's data.
    // Some items at the boundary may need a few extra bytes for type-resolved
    // element sizes that slightly exceed the stored data section.
    // During rebuild, cap to allocated size to prevent unintended growth.
    u32 elementSize = type ? getTypeByteSize(item.typeIndex) : 0;
    u32 requiredEnd = item.offset + elementSize * std::max(item.count, 1u);
    if (requiredEnd > (u32)mDataSection.size()) {
        // Only grow if the item is genuinely past the end (new item),
        // not just because type size overestimates the stored data
        if (item.offset + 1 > (u32)mDataSection.size()) {
            mDataSection.resize(requiredEnd, 0);
        } else {
            // Clamp to available space
            requiredEnd = (u32)mDataSection.size();
        }
    }

    u8* itemData = mDataSection.data() + item.offset;

    bool isCharType = (type && (type->name == "tT" || type->name == "char"));
    if (isCharType) elementSize = 1;

    // _bytes: direct byte overwrite
    if (itemNode["_bytes"] && itemNode["_bytes"].IsSequence()) {
        const YAML::Node& bytes = itemNode["_bytes"];
        for (size_t b = 0; b < bytes.size() && item.offset + b < mDataSection.size(); b++)
            itemData[b] = (u8)bytes[b].as<int>();
        return;
    }

    // _string: write string bytes + null terminator
    if (itemNode["_string"]) {
        std::string str = itemNode["_string"].as<std::string>();
        u32 maxLen = item.count > 0 ? item.count : (u32)str.size() + 1;
        memset(itemData, 0, maxLen);
        memcpy(itemData, str.c_str(), std::min((u32)str.size(), maxLen));
        return;
    }

    // _fields: encode record fields
    if (itemNode["_fields"] && type && effective && kind == KIND_RECORD) {
        const TypeDef* rt = effective;
        while (rt && rt->fields.empty() && rt->parentTypeId != 0)
            rt = getType(rt->parentTypeId);
        if (rt && !rt->fields.empty()) {
            encodeRecordYaml(itemNode["_fields"], itemData, *rt);
        }
        return;
    }

    // _elements: encode array of records/values/vectors
    if (itemNode["_elements"]) {
        const YAML::Node& elements = itemNode["_elements"];

        if (type && effective && kind == KIND_RECORD) {
            // Array of records
            const TypeDef* rt = effective;
            while (rt && rt->fields.empty() && rt->parentTypeId != 0)
                rt = getType(rt->parentTypeId);
            if (rt && elements.IsSequence()) {
                for (u32 e = 0; e < elements.size() && e < item.count; e++) {
                    u8* eData = itemData + e * elementSize;
                    if (item.offset + (e + 1) * elementSize <= mDataSection.size())
                        encodeRecordYaml(elements[e], eData, *rt);
                }
            }
        } else if (type && effective && kind == KIND_ARRAY &&
                   effective->subtypeId != 0 && effective->size > 0) {
            // Fixed-size inline arrays (vectors)
            const TypeDef* elemType = getType(effective->subtypeId);
            u32 elemSize = elemType ? elemType->size : 0;
            if (elemType && elemSize > 0) {
                u32 vecSize = (elementSize > 0 && elementSize < effective->size) ? elementSize : effective->size;
                u32 elemsPerVec = vecSize / elemSize;
                if (item.count == 1 || item.flags == 0x10) {
                    // Single vector
                    if (elements.IsSequence()) {
                        for (u32 vi = 0; vi < elemsPerVec && vi < elements.size(); vi++) {
                            if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                                float f = yamlToFloat(elements[vi]);
                                memcpy(itemData + vi * 4, &f, 4);
                            } else if (elemType->kind == KIND_INT && elemSize == 4) {
                                s32 val = elements[vi].as<s32>();
                                memcpy(itemData + vi * 4, &val, 4);
                            } else {
                                itemData[vi * elemSize] = (u8)elements[vi].as<int>();
                            }
                        }
                    }
                } else if (item.count > 0 && elements.IsSequence()) {
                    // Array of vectors
                    for (u32 e = 0; e < elements.size() && e < item.count; e++) {
                        u8* eData = itemData + e * vecSize;
                        if (item.offset + (e + 1) * vecSize > mDataSection.size()) break;
                        if (elements[e].IsSequence()) {
                            for (u32 vi = 0; vi < elemsPerVec && vi < elements[e].size(); vi++) {
                                if (elemType->kind == KIND_FLOAT && elemSize == 4) {
                                    float f = yamlToFloat(elements[e][vi]);
                                    memcpy(eData + vi * 4, &f, 4);
                                } else if (elemType->kind == KIND_INT && elemSize == 4) {
                                    s32 val = elements[e][vi].as<s32>();
                                    memcpy(eData + vi * 4, &val, 4);
                                } else {
                                    eData[vi * elemSize] = (u8)elements[e][vi].as<int>();
                                }
                            }
                        }
                    }
                }
            }
        } else if (type && effective &&
                   (kind == KIND_INT || kind == KIND_FLOAT || kind == KIND_BOOL)) {
            // Array of scalar values
            if (elements.IsSequence()) {
                for (u32 e = 0; e < elements.size() && e < item.count; e++) {
                    u8* eData = itemData + e * elementSize;
                    if (item.offset + (e + 1) * elementSize <= mDataSection.size())
                        encodeValueYaml(elements[e], eData, *effective);
                }
            }
        }
        return;
    }

    // _value: encode single scalar
    if (itemNode["_value"] && type && effective) {
        encodeValueYaml(itemNode["_value"], itemData, *effective);
        return;
    }
}

void hkTagfile::rebuildDataFromYaml(const YAML::Node& root) {
    const YAML::Node& items = root["items"];
    if (!items || !items.IsSequence()) return;

    // ---- Full data section rebuild ----
    size_t newItemCount = items.size();

    // Save old item offsets before modification (for pointer fixup intervals)
    std::vector<u32> oldOffsets(mItems.size(), 0);
    for (size_t i = 1; i < mItems.size(); i++)
        oldOffsets[i] = mItems[i].offset;

    // Compute old item gap sizes (offset intervals) for layout preservation
    std::vector<size_t> sortedByOff(mItems.size());
    for (size_t i = 0; i < mItems.size(); i++) sortedByOff[i] = i;
    std::sort(sortedByOff.begin(), sortedByOff.end(),
        [&](size_t a, size_t b) { return oldOffsets[a] < oldOffsets[b]; });
    std::vector<u32> oldItemSizes(mItems.size(), 0);
    for (size_t si = 0; si < sortedByOff.size(); si++) {
        size_t idx = sortedByOff[si];
        if (idx == 0) continue;
        // Find next non-zero offset after this item
        u32 nextOff = (u32)mDataSection.size();
        for (size_t sj = si + 1; sj < sortedByOff.size(); sj++) {
            size_t nidx = sortedByOff[sj];
            if (nidx > 0 && oldOffsets[nidx] > oldOffsets[idx]) {
                nextOff = oldOffsets[nidx];
                break;
            }
        }
        oldItemSizes[idx] = nextOff - oldOffsets[idx];
    }

    mItems.resize(newItemCount);

    std::vector<u32> itemSizes(newItemCount, 0);
    for (size_t i = 1; i < newItemCount; i++) {
        if (items[i].IsNull()) continue;

        if (items[i]["_typeIndex"])
            mItems[i].typeIndex = items[i]["_typeIndex"].as<u16>();
        if (items[i]["_flags"])
            mItems[i].flags = items[i]["_flags"].as<u8>();

        u32 elementSize = getTypeByteSize(mItems[i].typeIndex);
        const TypeDef* type = getType(mItems[i].typeIndex);
        bool isCharType = (type && (type->name == "tT" || type->name == "char"));
        if (isCharType) elementSize = 1;

        // Determine YAML-derived count for this item
        u32 yamlCount = mItems[i].count; // default to original
        bool countChanged = false;

        if (items[i]["_string"]) {
            std::string str = items[i]["_string"].as<std::string>();
            u32 newCount = (u32)(str.size() + 1);
            countChanged = (newCount != mItems[i].count);
            mItems[i].count = newCount;
            itemSizes[i] = mItems[i].count;
        } else if (items[i]["_bytes"] && items[i]["_bytes"].IsSequence()) {
            u32 byteCount = (u32)items[i]["_bytes"].size();
            u32 newCount = elementSize > 0 ? byteCount / elementSize : byteCount;
            countChanged = (newCount != mItems[i].count);
            itemSizes[i] = byteCount;
            mItems[i].count = newCount;
        } else if (items[i]["_elements"] && items[i]["_elements"].IsSequence()) {
            yamlCount = (u32)items[i]["_elements"].size();
            // Empty _elements with non-zero original count means the encoder
            // couldn't decode the data — preserve original count, not 0
            if (yamlCount == 0 && mItems[i].count > 0) {
                // Keep original count and size
            } else {
                FormatKind kind = resolveEffectiveKind(mItems[i].typeIndex);
                const TypeDef* effective = resolveEffectiveType(mItems[i].typeIndex);
                if (!effective) effective = type;
                if (kind == KIND_ARRAY && effective && effective->subtypeId != 0 && effective->size > 0) {
                    u32 vecSize = (elementSize > 0 && elementSize < effective->size) ? elementSize : effective->size;
                    bool isSingleVec = (yamlCount > 0 && !items[i]["_elements"][0].IsSequence());
                    if (isSingleVec) {
                        countChanged = (1 != mItems[i].count);
                        mItems[i].count = 1;
                        itemSizes[i] = vecSize;
                    } else {
                        countChanged = (yamlCount != mItems[i].count);
                        mItems[i].count = yamlCount;
                        itemSizes[i] = vecSize * yamlCount;
                    }
                } else {
                    countChanged = (yamlCount != mItems[i].count);
                    mItems[i].count = yamlCount;
                    itemSizes[i] = elementSize * yamlCount;
                }
            }
        } else if (items[i]["_fields"]) {
            mItems[i].count = 1;
            itemSizes[i] = elementSize;
        } else if (items[i]["_value"]) {
            mItems[i].count = 1;
            itemSizes[i] = elementSize;
        } else {
            itemSizes[i] = elementSize * mItems[i].count;
        }

        // If count hasn't changed and we have a known gap size from the original
        // layout, use that instead — type-derived sizes can be inaccurate for
        // complex types where the serialized size differs from the type definition.
        if (!countChanged && i < oldItemSizes.size() && oldItemSizes[i] > 0) {
            itemSizes[i] = oldItemSizes[i];
        }
    }

    // Compute new offsets using incremental shift to preserve original layout.
    // Items in offset-order get shifted by cumulative delta of size changes.
    std::vector<std::pair<u32, size_t>> offsetSorted; // (old_offset, index)
    for (size_t i = 1; i < newItemCount; i++) {
        if (items[i].IsNull()) continue;
        u32 oldOff = (i < oldOffsets.size()) ? oldOffsets[i] : 0;
        offsetSorted.push_back({oldOff, i});
    }
    std::sort(offsetSorted.begin(), offsetSorted.end());

    // Compute new offsets: keep items at their original positions.
    // Only items whose content grew beyond their gap get relocated
    // to overflow space at the end of the data section.
    std::vector<u32> newOffsets(newItemCount, 0);
    u32 overflowStart = (u32)mDataSection.size(); // append after original data
    u32 overflowCount = 0;
    for (auto& [oldOff, idx] : offsetSorted) {
        u32 oldGapSz = (idx < oldItemSizes.size()) ? oldItemSizes[idx] : 0;
        u32 newContentSz = itemSizes[idx];

        if (newContentSz <= oldGapSz) {
            // Fits in original gap - keep at original position
            newOffsets[idx] = oldOff;
        } else {
            // Overflow - relocate to end of data section
            overflowCount++;
            u32 alignment = getTypeAlignment(mItems[idx].typeIndex);
            if (alignment == 0) alignment = 1;
            overflowStart = (overflowStart + alignment - 1) & ~(alignment - 1);
            newOffsets[idx] = overflowStart;
            overflowStart += newContentSz;
        }
    }
    if (overflowCount > 0) {
        // Items that grew beyond their original gap were relocated to the end
    }

    u32 totalSize = overflowStart;
    // Pad DATA section to at least original size to keep TYPE/INDX at same positions
    if (mOrigDataPayloadSize > 0 && totalSize < mOrigDataPayloadSize)
        totalSize = mOrigDataPayloadSize;

    // Build per-item old→new offset mapping for pointer fixup
    struct ItemRemap { u32 oldOff; u32 newOff; u32 oldSize; };
    std::vector<ItemRemap> remaps;
    std::map<u32, u32> offsetMap; // old item start → new item start
    for (size_t i = 1; i < newItemCount; i++) {
        u32 oldOff = (i < oldOffsets.size()) ? oldOffsets[i] : 0;
        u32 oldSz = (i < oldItemSizes.size()) ? oldItemSizes[i] : itemSizes[i];
        offsetMap[oldOff] = newOffsets[i];
        remaps.push_back({oldOff, newOffsets[i], oldSz});
    }
    // Sort remaps by old offset for binary search
    std::sort(remaps.begin(), remaps.end(),
        [](const ItemRemap& a, const ItemRemap& b) { return a.oldOff < b.oldOff; });

    std::vector<u8> newData(totalSize, 0);

    // Copy original data where possible (use gap sizes to preserve padding)
    for (size_t i = 1; i < newItemCount; i++) {
        if (items[i].IsNull()) continue;
        u32 oldOff = (i < oldOffsets.size()) ? oldOffsets[i] : 0;
        u32 newOff = newOffsets[i];
        // Copy the full old gap (content + padding) to preserve inter-item bytes
        u32 oldGap = (i < oldItemSizes.size()) ? oldItemSizes[i] : 0;
        u32 copySize = oldGap;
        if (oldOff + copySize > mDataSection.size())
            copySize = (oldOff < mDataSection.size()) ? (u32)(mDataSection.size() - oldOff) : 0;
        if (newOff + copySize > newData.size())
            copySize = (newOff < newData.size()) ? (u32)(newData.size() - newOff) : 0;
        if (copySize > 0) {
            memcpy(newData.data() + newOff, mDataSection.data() + oldOff, copySize);
        }
    }

    for (size_t i = 1; i < newItemCount; i++)
        mItems[i].offset = newOffsets[i];

    mDataSection = std::move(newData);

    // Apply YAML values on top
    for (size_t i = 1; i < newItemCount; i++) {
        if (items[i].IsNull()) continue;
        applyYamlItemToData(items[i], i);
    }

    // Fix up patch offsets and pointer values using sorted interval lookup
    auto remapOffset = [&](u32 off) -> u32 {
        // Binary search: find the last remap whose oldOff <= off
        auto it = std::upper_bound(remaps.begin(), remaps.end(), off,
            [](u32 val, const ItemRemap& r) { return val < r.oldOff; });
        if (it != remaps.begin()) {
            --it;
            if (off >= it->oldOff && off < it->oldOff + it->oldSize)
                return it->newOff + (off - it->oldOff);
        }
        return off; // unmapped offset stays as-is
    };

    for (auto& patch : mPatches) {
        for (auto& off : patch.offsets) {
            u32 newPatchOff = remapOffset(off);

            // Fix pointer value at this location
            if (newPatchOff + 8 <= mDataSection.size()) {
                u64 ptrVal;
                memcpy(&ptrVal, mDataSection.data() + newPatchOff, 8);
                if (ptrVal != 0) {
                    auto it = offsetMap.find((u32)ptrVal);
                    if (it != offsetMap.end()) {
                        u64 newPtr = it->second;
                        memcpy(mDataSection.data() + newPatchOff, &newPtr, 8);
                    }
                }
            }
            off = newPatchOff;
        }
    }

    // Pad DATA to 4-byte alignment (parser skips padding in 4-byte increments)
    while (mDataSection.size() % 4 != 0) mDataSection.push_back(0);
    u32 dataFlags = mDataSizeRaw & 0xC0000000;
    mDataSizeRaw = ((u32)mDataSection.size() + 8) | dataFlags;

}

// ============================================================================
// YAML Deserialization
// ============================================================================

bool hkTagfile::fromYaml(const YAML::Node& root) {
    mSdkVersion = root["sdk_version"].as<std::string>();

    // Restore from metadata section
    const YAML::Node& meta = root["_metadata"];
    mSdkvSizeRaw = meta["sdkv_size_raw"].as<u32>();
    mDataSizeRaw = meta["data_size_raw"].as<u32>();
    mIndxTagOffset = meta["indx_tag_offset"].as<u32>();
    mOrigDataPayloadSize = meta["orig_data_payload_size"]
        ? meta["orig_data_payload_size"].as<u32>() : 0;

    // Restore TYPE section from base64
    mTypeSection = base64Decode(meta["type_section"].as<std::string>());
    parseTypeSection();

    // Restore DATA section from base64
    mDataSection = base64Decode(meta["data_section"].as<std::string>());

    // Restore items from layout metadata
    const YAML::Node& layout = meta["item_layout"];
    mItems.resize(layout.size());
    for (size_t i = 0; i < layout.size(); i++) {
        mItems[i].typeIndex = layout[i]["typeIndex"].as<u16>();
        mItems[i].flags = layout[i]["flags"].as<u8>();
        mItems[i].offset = layout[i]["offset"].as<u32>();
        mItems[i].count = layout[i]["count"].as<u32>();
    }

    // Restore patches
    mPatches.clear();
    if (root["patches"]) {
        for (const auto& pn : root["patches"]) {
            PatchEntry patch;
            patch.typeIndex = pn["typeIndex"].as<u32>();
            for (const auto& off : pn["offsets"])
                patch.offsets.push_back(off.as<u32>());
            mPatches.push_back(std::move(patch));
        }
    }

    // Apply YAML item values back to data section (supports editing and expansion)
    rebuildDataFromYaml(root);

    return true;
}

// ============================================================================
// Binary Writing
// ============================================================================

void hkTagfile::writeToBinary(std::vector<u8>& out) const {
    out.clear();
    out.reserve(256 * 1024);

    auto writeU32BE = [&](u32 pos, u32 val) {
        out[pos] = (u8)(val >> 24);
        out[pos+1] = (u8)(val >> 16);
        out[pos+2] = (u8)(val >> 8);
        out[pos+3] = (u8)val;
    };

    auto appendU32BE = [&](u32 val) {
        out.push_back((u8)(val >> 24));
        out.push_back((u8)(val >> 16));
        out.push_back((u8)(val >> 8));
        out.push_back((u8)val);
    };

    auto appendStr = [&](const char* str, size_t len) {
        for (size_t i = 0; i < len; i++) out.push_back(str[i]);
    };

    auto appendAlign = [&](size_t alignment) {
        while (out.size() % alignment != 0) out.push_back(0);
    };

    // TAG0 header placeholder
    size_t tag0Start = out.size();
    appendU32BE(0); // placeholder for TAG0 size
    appendStr("TAG0", 4);

    // SDKV section (preserve original size+flags)
    appendU32BE(mSdkvSizeRaw);
    appendStr("SDKV", 4);
    for (char c : mSdkVersion) out.push_back(c);
    // Pad SDKV content to match original section size
    u32 sdkvContentSize = (mSdkvSizeRaw & 0x3FFFFFFF) - 8;
    while (out.size() < tag0Start + 8 + (mSdkvSizeRaw & 0x3FFFFFFF))
        out.push_back(0);

    // DATA section (preserve original size+flags format)
    // Pad DATA content to 4-byte alignment so subsequent sections stay aligned
    // (the parser skips zero-padding in 4-byte increments between sections)
    size_t dataContentSize = mDataSection.size();
    size_t dataContentAligned = (dataContentSize + 3) & ~(size_t)3;
    u32 dataSizeWithHeader = (u32)(dataContentAligned + 8);
    u32 dataFlags = mDataSizeRaw & 0xC0000000;
    if (mDataSizeRaw == 0) dataFlags = 0x40000000;
    appendU32BE(dataSizeWithHeader | dataFlags);
    appendStr("DATA", 4);
    out.insert(out.end(), mDataSection.begin(), mDataSection.end());
    // Pad DATA content to alignment
    while (out.size() < tag0Start + 8 + (mSdkvSizeRaw & 0x3FFFFFFF) + dataSizeWithHeader)
        out.push_back(0);

    // TYPE section (verbatim - includes internal TPAD for alignment)
    out.insert(out.end(), mTypeSection.begin(), mTypeSection.end());

    // Pad to reach original INDX offset if known
    if (mIndxTagOffset > 0) {
        size_t targetPos = tag0Start + mIndxTagOffset;
        while (out.size() < targetPos) out.push_back(0);
    }

    // INDX section (immediately follows TYPE + padding)
    size_t indxStart = out.size();
    appendU32BE(0); // placeholder for INDX size
    appendStr("INDX", 4);

    // ITEM subsection
    size_t itemStart = out.size();
    appendU32BE(0); // placeholder for ITEM size
    appendStr("ITEM", 4);

    for (const auto& item : mItems) {
        // typeIndex (u16) + pad (u8) + flags (u8)
        out.push_back((u8)(item.typeIndex & 0xFF));
        out.push_back((u8)(item.typeIndex >> 8));
        out.push_back(0); // pad
        out.push_back(item.flags);
        // dataOffset (u32 LE)
        u32 off = item.offset;
        out.push_back((u8)(off)); out.push_back((u8)(off >> 8));
        out.push_back((u8)(off >> 16)); out.push_back((u8)(off >> 24));
        // count (u32 LE)
        u32 cnt = item.count;
        out.push_back((u8)(cnt)); out.push_back((u8)(cnt >> 8));
        out.push_back((u8)(cnt >> 16)); out.push_back((u8)(cnt >> 24));
    }

    // Write ITEM size (includes size+magic, with flag bit)
    size_t itemSize = out.size() - itemStart;
    writeU32BE((u32)itemStart, (u32)itemSize | 0x40000000);

    // PTCH subsection
    size_t ptchStart = out.size();
    appendU32BE(0); // placeholder for PTCH size
    appendStr("PTCH", 4);

    for (const auto& patch : mPatches) {
        u32 ti = patch.typeIndex;
        out.push_back((u8)(ti)); out.push_back((u8)(ti >> 8));
        out.push_back((u8)(ti >> 16)); out.push_back((u8)(ti >> 24));
        u32 cnt = (u32)patch.offsets.size();
        out.push_back((u8)(cnt)); out.push_back((u8)(cnt >> 8));
        out.push_back((u8)(cnt >> 16)); out.push_back((u8)(cnt >> 24));
        for (u32 off : patch.offsets) {
            out.push_back((u8)(off)); out.push_back((u8)(off >> 8));
            out.push_back((u8)(off >> 16)); out.push_back((u8)(off >> 24));
        }
    }

    size_t ptchSize = out.size() - ptchStart;
    writeU32BE((u32)ptchStart, (u32)ptchSize | 0x40000000);

    // Write INDX size
    size_t indxSize = out.size() - indxStart;
    writeU32BE((u32)indxStart, (u32)indxSize);

    // Write TAG0 size (no trailing alignment - INDX ends the TAG file)
    size_t tag0Size = out.size() - tag0Start;
    writeU32BE((u32)tag0Start, (u32)tag0Size);
}

} // namespace Havok
