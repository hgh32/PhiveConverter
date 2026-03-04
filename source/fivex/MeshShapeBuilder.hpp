#pragma once

#include "types.h"
#include "havok/hkTypes.h"
#include "havok/hknpShape.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>

namespace FiveX {

struct BuildNode {
    hkcdFourAabb aabb;
    hkUint32 data[4] = {};
    bool isLeaf = false;
    bool isActive = false;
};

struct TempGeometrySection {
    hkAabb domain;
    hkAabb originalDomain;
    hkAabb refitDomain;
    std::vector<hknpAabb8TreeNode> sectionBvh;
    std::vector<hknpMeshShape::GeometrySection::Primitive> primitives;
    std::vector<float> verticesFlat; // x,y,z triples
    std::vector<hknpMeshShape::GeometrySection::Vertex16_3> quantizedVertices;
    std::vector<hkUint16> shapeTags;
    std::vector<bool> primitiveDisableAllEdges;
    std::vector<hkUint8> interiorBitField;
    hkUint32 sectionOffset[3] = {};
    hkFloat3 bitScale8Inv = {};
    hkInt16 bitOffset[3] = {};
};

struct MeshShapeBuilder {
    static hknpMeshShape* build(const hkGeometry& geometry, float maxVertexError = 1e-3f);

    static void buildSurfaceGeometry(const hknpMeshShape* shape, hkGeometry& geometryOut);
};

}
