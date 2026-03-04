#include "fivex/MeshShapeBuilder.hpp"
#include <cassert>
#include <numeric>
#include <unordered_map>

namespace FiveX {

static float calcBitScale(float maxVertexError) {
    float stepSize = maxVertexError * 2.0f;
    float log2Step = std::floor(std::log2f(stepSize));
    return std::exp2f(-log2Step);
}

static void initVertexConversionUtil(hknpMeshShapeVertexConversionUtil& util, float maxVertexError) {
    float scale = calcBitScale(maxVertexError);
    for (int i = 0; i < 3; i++) {
        util.m_bitScale16[i] = scale;
        util.m_bitScale16Inv[i] = 1.0f / scale;
    }
    util.m_bitScale16[3] = 0;
    util.m_bitScale16Inv[3] = 0;
}

static float calcMaxAllowedSectionExtent(float maxVertexError) {
    float invScale = 1.0f / calcBitScale(maxVertexError);
    return (65536.0f - 2.0f) * invScale;
}

static void getSectionOffset(const hknpMeshShapeVertexConversionUtil& util, const hkAabb& domain, hkUint32 sectionOffset[3]) {
    for (int i = 0; i < 3; i++) {
        float scaled = domain.m_min[i] * util.m_bitScale16[i];
        sectionOffset[i] = (hkUint32)(hkInt32)std::round(scaled);
    }
}

static void packVertex(const hknpMeshShapeVertexConversionUtil& util, const float* vertex, const hkUint32 sectionOffset[3], hkUint16* out) {
    for (int i = 0; i < 3; i++) {
        float scaled = vertex[i] * util.m_bitScale16[i];
        int quantized = (int)std::round(scaled) - (hkInt32)sectionOffset[i];
        out[i] = (hkUint16)std::max(0, std::min(65535, quantized));
    }
}

static void unpackVertex(const hknpMeshShapeVertexConversionUtil& util, const hkUint16* packed, const hkUint32 sectionOffset[3], float* out) {
    for (int i = 0; i < 3; i++) {
        int val = (int)packed[i] + (hkInt32)sectionOffset[i];
        out[i] = (float)val * util.m_bitScale16Inv[i];
    }
}

struct Aabb8Quantizer {
    float bitScaleInv[3];
    int bitOffset[3];
    float bitScale[3];

    Aabb8Quantizer(const hkAabb& domain) {
        hkAabb expanded = domain;

        for (int i = 0; i < 3; i++) {
            if (expanded.m_min[i] == expanded.m_max[i]) {
                expanded.m_min[i] -= FLT_EPSILON;
                expanded.m_max[i] += FLT_EPSILON;
            }
        }

        for (int i = 0; i < 3; i++) {
            float extent = expanded.m_max[i] - expanded.m_min[i];
            float expand = extent / 255.0f;
            float minAbs = std::abs(expanded.m_min[i]) / 255.0f;
            expand = std::max(expand, minAbs);
            expanded.m_max[i] += expand;
            expanded.m_min[i] -= expand;
        }

        for (int i = 0; i < 3; i++) {
            float span = expanded.m_max[i] - expanded.m_min[i];
            bitScale[i] = 255.0f / span;
            float rangeMin = expanded.m_min[i] * bitScale[i];
            rangeMin = std::round(rangeMin);
            bitOffset[i] = (int)rangeMin;
            bitScaleInv[i] = 1.0f / bitScale[i];
        }
    }

    static hkUint8 clampU8(int v) { return (hkUint8)std::max(0, std::min(255, v)); }

    void convertAabb(const hkAabb& aabbF, hkUint8* lx, hkUint8* hx, hkUint8* ly, hkUint8* hy, hkUint8* lz, hkUint8* hz) const {
        *lx = clampU8(quantizeMin(aabbF.m_min[0], 0));
        *hx = clampU8(quantizeMax(aabbF.m_max[0], 0));
        *ly = clampU8(quantizeMin(aabbF.m_min[1], 1));
        *hy = clampU8(quantizeMax(aabbF.m_max[1], 1));
        *lz = clampU8(quantizeMin(aabbF.m_min[2], 2));
        *hz = clampU8(quantizeMax(aabbF.m_max[2], 2));
    }

private:
    int quantizeMin(float val, int axis) const {
        float scaled = bitScale[axis] * val;
        float rounded = scaled < 0 ? -std::ceil(-scaled) : scaled;
        int compressed = (int)rounded;
        int result = compressed - bitOffset[axis];
        float restored = (float)compressed * bitScaleInv[axis];
        if (restored > val) result--;
        return result;
    }

    int quantizeMax(float val, int axis) const {
        float scaled = bitScale[axis] * val;
        float rounded = scaled > 0 ? std::ceil(scaled) : scaled;
        int compressed = (int)rounded;
        int result = compressed - bitOffset[axis];
        float restored = (float)compressed * bitScaleInv[axis];
        if (restored < val) result++;
        return result;
    }
};

static void setAabb8(hknpTransposedFourAabbs8& node, int index, hkUint8 lx, hkUint8 hx, hkUint8 ly, hkUint8 hy, hkUint8 lz, hkUint8 hz) {
    ((hkUint8*)&node.m_lx)[index] = lx;
    ((hkUint8*)&node.m_hx)[index] = hx;
    ((hkUint8*)&node.m_ly)[index] = ly;
    ((hkUint8*)&node.m_hy)[index] = hy;
    ((hkUint8*)&node.m_lz)[index] = lz;
    ((hkUint8*)&node.m_hz)[index] = hz;
}

static void clearAabb8(hknpTransposedFourAabbs8& node, int index) {
    setAabb8(node, index, 0xFF, 0, 0xFF, 0, 0xFF, 0);
}

static void getAabb8(const hknpTransposedFourAabbs8& node, int index, hkUint8& lx, hkUint8& hx, hkUint8& ly, hkUint8& hy, hkUint8& lz, hkUint8& hz) {
    lx = ((const hkUint8*)&node.m_lx)[index];
    hx = ((const hkUint8*)&node.m_hx)[index];
    ly = ((const hkUint8*)&node.m_ly)[index];
    hy = ((const hkUint8*)&node.m_hy)[index];
    lz = ((const hkUint8*)&node.m_lz)[index];
    hz = ((const hkUint8*)&node.m_hz)[index];
}

static bool isAabb8Valid(const hknpTransposedFourAabbs8& node, int index) {
    return ((const hkUint8*)&node.m_lx)[index] <= ((const hkUint8*)&node.m_hx)[index];
}

static void compoundAabb8(const hknpTransposedFourAabbs8& node, hkUint8& olx, hkUint8& ohx, hkUint8& oly, hkUint8& ohy, hkUint8& olz, hkUint8& ohz) {
    olx = 255; ohx = 0; oly = 255; ohy = 0; olz = 255; ohz = 0;
    for (int i = 0; i < 4; i++) {
        if (!isAabb8Valid(node, i)) continue;
        hkUint8 lx, hx, ly, hy, lz, hz;
        getAabb8(node, i, lx, hx, ly, hy, lz, hz);
        olx = std::min(olx, lx); ohx = std::max(ohx, hx);
        oly = std::min(oly, ly); ohy = std::max(ohy, hy);
        olz = std::min(olz, lz); ohz = std::max(ohz, hz);
    }
}

static void swapAabb8Children(hknpAabb8TreeNode& node, int i1, int i2) {
    hkUint8 lx1, hx1, ly1, hy1, lz1, hz1;
    hkUint8 lx2, hx2, ly2, hy2, lz2, hz2;
    getAabb8(node, i1, lx1, hx1, ly1, hy1, lz1, hz1);
    getAabb8(node, i2, lx2, hx2, ly2, hy2, lz2, hz2);
    setAabb8(node, i1, lx2, hx2, ly2, hy2, lz2, hz2);
    setAabb8(node, i2, lx1, hx1, ly1, hy1, lz1, hz1);
    std::swap(node.m_data[i1], node.m_data[i2]);
}

static bool isAabb8TreeNodeLeaf(const hknpAabb8TreeNode& node) {
    return node.m_data[2] > node.m_data[3];
}

static void configureAsLeafOrInternal(hknpAabb8TreeNode& node, bool targetIsLeaf) {
    if (isAabb8TreeNodeLeaf(node) != targetIsLeaf) {
        if (isAabb8Valid(node, 2) && isAabb8Valid(node, 3)) {
            swapAabb8Children(node, 2, 3);
        } else if (isAabb8Valid(node, 2)) {
            if (targetIsLeaf) {
                if (node.m_data[2] == 0)
                    swapAabb8Children(node, 1, 2);
                node.m_data[3] = node.m_data[2] - 1;
            } else {
                node.m_data[3] = node.m_data[2];
            }
        } else {
            if (targetIsLeaf)
                node.m_data[2] = 1;
        }
    }
}

static bool isAabb8TreeNodeChildValid(const hknpAabb8TreeNode& node, int i) {
    return isAabb8Valid(node, i);
}

static hkUint8 getAabb8TreeNodePrimitiveIndex(const hknpAabb8TreeNode& node, int i) {
    return node.m_data[i];
}

static hkUint64 spatialHash(float x, float y, float z) {
    union { float f; hkUint32 u; } ux, uy, uz;
    ux.f = x; uy.f = y; uz.f = z;
    const hkUint64 p1 = 73856093, p2 = 19349663, p3 = 83492791;
    return ((hkUint64)ux.u * p1) ^ ((hkUint64)uy.u * p2) ^ ((hkUint64)uz.u * p3);
}


template<typename T, typename Cmp>
static void quickSortRecursive(T* pArr, int d, int h, Cmp cmpLess) {
    int i, j;
    T str;
begin:
    i = h;
    j = d;
    str = pArr[(d + h) >> 1];
    do {
        while (cmpLess(pArr[j], str)) j++;
        while (cmpLess(str, pArr[i])) i--;
        if (i >= j) {
            if (i != j) std::swap(pArr[i], pArr[j]);
            i--;
            j++;
        }
    } while (j <= i);
    if (d < i) quickSortRecursive(pArr, d, i, cmpLess);
    if (j < h) { d = j; goto begin; }
}
template<typename T, typename Cmp>
static void quickSort(T* pArr, int iSize, Cmp cmpLess) {
    if (iSize > 1) quickSortRecursive(pArr, 0, iSize - 1, cmpLess);
}

static hkUint32 floatToOrderedUint(float f) {
    hkUint32 ui;
    memcpy(&ui, &f, 4);
    return (hkUint32((hkInt32)ui >> 31) | 0x80000000u) ^ ui;
}

struct SweepAabb {
    hkUint32 minVal[4];
    hkUint32 maxVal[4];
    hkUint32 getKey() const { return minVal[3]; }
    void setKey(hkUint32 k) { minVal[3] = k; }
};

struct RadixEntry {
    hkUint32 key;
    hkUint32 idx;
    bool operator<(const RadixEntry& o) const { return key < o.key; }
};

static void sortQuickStack(RadixEntry* base, int numMem) {
    if (numMem <= 1) return;
    const int maxDepth = sizeof(void*) * 8;
    RadixEntry *lbStack[maxDepth], *ubStack[maxDepth];
    lbStack[0] = base;
    ubStack[0] = base + (numMem - 1);
    for (int stackPos = 0; stackPos >= 0; stackPos--) {
        RadixEntry* lb = lbStack[stackPos];
        RadixEntry* ub = ubStack[stackPos];
        while (true) {
            RadixEntry* j = lb;
            RadixEntry* i = ub;
            RadixEntry pivot = lb[(ub - lb) >> 1];
            do {
                while (j->key < pivot.key) j++;
                while (pivot.key < i->key) i--;
                if (i >= j) {
                    if (i != j) std::swap(*i, *j);
                    i--; j++;
                }
            } while (j <= i);
            if (lb < i) {
                if (j < ub) {
                    if (((char*)ub - (char*)j) > ((char*)i - (char*)lb)) {
                        lbStack[stackPos] = j;
                        ubStack[stackPos++] = ub;
                        ub = i;
                    } else {
                        lbStack[stackPos] = lb;
                        ubStack[stackPos++] = i;
                        lb = j;
                    }
                    continue;
                } else { ub = i; continue; }
            } else {
                if (j < ub) { lb = j; continue; }
            }
            break;
        }
    }
}

static void sortLsb2Hsb(RadixEntry* data, hkUint32 numObjects, RadixEntry* buffer,
                         int sortKeySize, int lsbByteOffset, int outputBufferIndex) {
    hkUint32 hist[260];
    hkUint32 scatter[260];

    RadixEntry* source = data;
    RadixEntry* dest = buffer;

    int off0 = lsbByteOffset;
    memset(hist, 0, sizeof(hist));
    for (hkUint32 i = 0; i < numObjects; i++)
        hist[((const hkUint8*)&source[i])[off0]]++;

    for (int k = 0; k < sortKeySize - 1; k++) {
        int off1 = off0 + 1;
        if (hist[0] != numObjects && hist[255] != numObjects) {
            hkUint32 sum = 0;
            memcpy(scatter, hist, sizeof(hist));
            for (int b = 0; b < 256; b++) { hkUint32 c = scatter[b]; scatter[b] = sum; sum += c; }
            memset(hist, 0, sizeof(hist));
            for (hkUint32 i = 0; i < numObjects; i++) {
                int t0 = ((const hkUint8*)&source[i])[off0];
                int t1 = ((const hkUint8*)&source[i])[off1];
                dest[scatter[t0]++] = source[i];
                hist[t1]++;
            }
            std::swap(source, dest);
        } else {
            memset(hist, 0, sizeof(hist));
            for (hkUint32 i = 0; i < numObjects; i++)
                hist[((const hkUint8*)&source[i])[off1]]++;
        }
        off0 = off1;
    }

    if (hist[0] != numObjects && hist[255] != numObjects) {
        hkUint32 sum = 0;
        memcpy(scatter, hist, sizeof(hist));
        for (int b = 0; b < 256; b++) { hkUint32 c = scatter[b]; scatter[b] = sum; sum += c; }
        for (hkUint32 i = 0; i < numObjects; i++) {
            int t0 = ((const hkUint8*)&source[i])[off0];
            dest[scatter[t0]++] = source[i];
        }
        std::swap(source, dest);
    }

    if ((dest == data) ^ (outputBufferIndex == 1)) {
        for (hkUint32 i = 0; i < numObjects; i++)
            dest[i] = source[i];
    }
}

static int countLeadingZeros32(hkUint32 v) {
    if (v == 0) return 32;
    unsigned long idx;
    _BitScanReverse(&idx, v);
    return 31 - (int)idx;
}

static void sortRadix(RadixEntry* data, int n, RadixEntry* buffer) {
    const int QUICK_SORT_THRESHOLD = 32;
    const int SMALL_ARRAY_THRESHOLD = 8192;
    const int OPTIMAL_LSB2HSB_SIZE = 1024;
    const int SORT_KEY_SIZE = 4;
    const int LSB_BYTE_OFFSET = 0;

    if (n <= QUICK_SORT_THRESHOLD) {
        sortQuickStack(data, n);
        return;
    }
    if (n <= SMALL_ARRAY_THRESHOLD) {
        sortLsb2Hsb(data, n, buffer, SORT_KEY_SIZE, LSB_BYTE_OFFSET, 0);
        return;
    }

    hkUint32 valueOr = 0, valueAnd = ~(hkUint32)0;
    for (int i = 0; i < n; i++) {
        valueOr |= data[i].key;
        valueAnd &= data[i].key;
    }
    int leadingZeros = countLeadingZeros32(valueOr);
    int leadingOnes = countLeadingZeros32(~valueAnd);
    int numLeadingZeros = (leadingZeros > leadingOnes) ? leadingZeros : leadingOnes;

    int shift = 24 - numLeadingZeros;
    hkUint32 hist[260];
    memset(hist, 0, sizeof(hist));
    for (int i = 0; i < n; i++) {
        hkUint32 sortKey = (data[i].key >> shift) & 0xFF;
        hist[sortKey]++;
    }

    hkUint32 hsum[260];
    memset(hsum, 0, sizeof(hsum));
    hkUint32 runningSum = 0;
    for (int i = 0; i < 256; i++) {
        hsum[i] = runningSum;
        runningSum += hist[i];
    }
    hsum[256] = runningSum;

    hkUint32 scatterIdx[260];
    memcpy(scatterIdx, hsum, sizeof(hsum));
    for (int i = 0; i < n; i++) {
        hkUint32 sortKey = (data[i].key >> shift) & 0xFF;
        buffer[scatterIdx[sortKey]++] = data[i];
    }

    int splitThreshold = 1 << (numLeadingZeros & 7);
    struct WorkItem { hkUint16 start, end; };
    WorkItem workQueue[256];
    int numWorkItems = 0;

    struct StackItem { int s, e; };
    StackItem stack[32];
    stack[0] = {0, 256};
    int stackSize = 1;
    while (stackSize > 0) {
        StackItem si = stack[--stackSize];
        int s = si.s, e = si.e;
        while (true) {
            int num = (int)(hsum[e] - hsum[s]);
            if (num == 0) break;
            int span = e - s;
            if ((num <= OPTIMAL_LSB2HSB_SIZE && (span > 4 || span <= splitThreshold)) || span == 1) {
                workQueue[numWorkItems++] = { (hkUint16)s, (hkUint16)e };
                break;
            }
            int mid = (s + e) >> 1;
            int numA = (int)(hsum[mid] - hsum[s]);
            int numB = num - numA;
            if (numA > numB) {
                stack[stackSize++] = {s, mid};
                s = mid;
            } else {
                stack[stackSize++] = {mid, e};
                e = mid;
            }
        }
    }

    for (int w = 0; w < numWorkItems; w++) {
        hkUint32 start = hsum[workQueue[w].start];
        hkUint32 num = hsum[workQueue[w].end] - start;
        if (num > (hkUint32)QUICK_SORT_THRESHOLD) {
            RadixEntry* s = &buffer[start];
            RadixEntry* d = &data[start];
            int keySize = SORT_KEY_SIZE - (numLeadingZeros >> 3);
            int nlzBits = numLeadingZeros & 7;
            int span = workQueue[w].end - workQueue[w].start;
            if (span <= (1 << nlzBits))
                keySize--;
            sortLsb2Hsb(s, num, d, keySize, LSB_BYTE_OFFSET, 1);
        } else {
            RadixEntry* s = &buffer[start];
            RadixEntry* d = &data[start];
            for (hkUint32 k = 0; k < num; k++)
                d[k] = s[k];
            if (num >= 2)
                sortQuickStack(d, (int)num);
        }
    }
}

static void weldDuplicateVertices(hkGeometry& geometry) {
    int numVerts = geometry.m_numVertices;
    int numTris = geometry.m_numTriangles;
    if (numVerts <= 0) return;

    std::vector<SweepAabb> aabbs(numVerts + 4);
    for (int i = 0; i < numVerts; i++) {
        const hkVector4& v = geometry.m_vertices[i];
        for (int j = 0; j < 3; j++) {
            aabbs[i].minVal[j] = floatToOrderedUint(v[j]) >> 1;
            aabbs[i].maxVal[j] = (floatToOrderedUint(v[j]) >> 1) + 1;
        }
        aabbs[i].setKey((hkUint32)i);
    }

    std::vector<RadixEntry> sortArr(numVerts);
    std::vector<RadixEntry> sortBuf(numVerts);
    for (int i = 0; i < numVerts; i++) {
        sortArr[i].key = aabbs[i].minVal[0];
        sortArr[i].idx = (hkUint32)i;
    }
    sortRadix(sortArr.data(), numVerts, sortBuf.data());

    std::vector<SweepAabb> sortedAabbs(numVerts + 4);
    for (int i = 0; i < numVerts; i++)
        sortedAabbs[i] = aabbs[sortArr[i].idx];
    memcpy(aabbs.data(), sortedAabbs.data(), numVerts * sizeof(SweepAabb));

    std::vector<int> remap(numVerts, -1);
    std::vector<hkVector4> uniqueVerts;
    uniqueVerts.reserve(numVerts);

    for (int current = 0; current < numVerts; current++) {
        hkUint32 currentKey = aabbs[current].getKey();
        if (currentKey == 0xFFFFFFFF) continue;

        const hkVector4& currentPos = geometry.m_vertices[currentKey];
        remap[currentKey] = (int)uniqueVerts.size();
        hkVector4 v; v.set(currentPos[0], currentPos[1], currentPos[2], 0);
        uniqueVerts.push_back(v);

        for (int potential = current + 1;
             potential < numVerts && aabbs[potential].minVal[0] <= aabbs[current].maxVal[0];
             potential++) {
            hkUint32 potentialKey = aabbs[potential].getKey();
            if (potentialKey == 0xFFFFFFFF) continue;

            const hkVector4& potentialPos = geometry.m_vertices[potentialKey];
            float dx = currentPos[0] - potentialPos[0];
            float dy = currentPos[1] - potentialPos[1];
            float dz = currentPos[2] - potentialPos[2];
            if (dx * dx + dy * dy + dz * dz <= 0.0f) {
                remap[potentialKey] = remap[currentKey];
                aabbs[potential].setKey(0xFFFFFFFF);
            }
        }
    }

    delete[] geometry.m_vertices;
    geometry.m_numVertices = (int)uniqueVerts.size();
    geometry.m_vertices = new hkVector4[uniqueVerts.size()];
    memcpy(geometry.m_vertices, uniqueVerts.data(), uniqueVerts.size() * sizeof(hkVector4));

    for (int i = 0; i < numTris; i++) {
        geometry.m_triangles[i].m_a = remap[geometry.m_triangles[i].m_a];
        geometry.m_triangles[i].m_b = remap[geometry.m_triangles[i].m_b];
        geometry.m_triangles[i].m_c = remap[geometry.m_triangles[i].m_c];
    }

    // Remove degenerate triangles (where welding collapsed vertices)
    int writeIdx = 0;
    for (int i = 0; i < numTris; i++) {
        auto& t = geometry.m_triangles[i];
        if (t.m_a != t.m_b && t.m_a != t.m_c && t.m_b != t.m_c) {
            geometry.m_triangles[writeIdx++] = t;
        }
    }
    geometry.m_numTriangles = writeIdx;
}

struct MeshPrimitive {
    int verts[4];
    int shapeTag;
    bool isQuad;
    bool disableAllEdges;
};

struct HalfEdge {
    int triangle;
    int edgeIdx;
};

struct EdgeLink {
    int triA, edgeA;
    int triB, edgeB;
    float quality;
};

static void createQuadDominantGeometry(
    const hkGeometry& geometry,
    const std::vector<hkUint16>& shapeTags,
    float maxExtent,
    std::vector<MeshPrimitive>& outPrimitives)
{
    int numTris = geometry.m_numTriangles;
    int numVerts = geometry.m_numVertices;

    auto triVert = [&](int tri, int v) -> int {
        const auto& t = geometry.m_triangles[tri];
        if (v == 0) return t.m_a;
        if (v == 1) return t.m_b;
        return t.m_c;
    };

    // Build connectivity
    struct ConnEdge {
        int triangle;
        int start;
    };

    std::vector<int> vertCardinality(numVerts, 0);
    std::vector<int> vertFirstEdge(numVerts, 0);

    for (int t = 0; t < numTris; t++) {
        const auto& tri = geometry.m_triangles[t];
        bool valid = tri.m_a != tri.m_b && tri.m_b != tri.m_c && tri.m_c != tri.m_a;
        if (valid) {
            vertCardinality[tri.m_a]++;
            vertCardinality[tri.m_b]++;
            vertCardinality[tri.m_c]++;
        }
    }

    int numEdges = 0;
    for (int v = 0; v < numVerts; v++) {
        vertFirstEdge[v] = vertCardinality[v] ? numEdges : 0;
        numEdges += vertCardinality[v];
    }

    std::vector<ConnEdge> edges(numEdges);
    struct TriLinks { ConnEdge links[3]; bool valid[3]; };
    std::vector<TriLinks> triLinks(numTris);
    for (int t = 0; t < numTris; t++)
        for (int i = 0; i < 3; i++) { triLinks[t].valid[i] = false; triLinks[t].links[i] = {-1, -1}; }

    // insert edge, then search only already inserted edges, set both link directions
    std::vector<int> counters(numVerts, 0);
    for (int t = 0; t < numTris; t++) {
        const auto& tri = geometry.m_triangles[t];
        bool valid = tri.m_a != tri.m_b && tri.m_b != tri.m_c && tri.m_c != tri.m_a;
        if (!valid) continue;
        const int verts[3] = {tri.m_a, tri.m_b, tri.m_c};
        for (int i = 2, j = 0; j < 3; i = j++) {
            int vi = verts[i];
            int vj = verts[j];

            int idx = vertFirstEdge[vi] + counters[vi]++;
            edges[idx] = {t, i};

            int count = counters[vj];
            for (int k = 0; k < count; k++) {
                const ConnEdge& otherEdge = edges[vertFirstEdge[vj] + k];
                int otherEnd = triVert(otherEdge.triangle, (otherEdge.start + 1) % 3);
                if (otherEnd == vi) {
                    triLinks[t].links[i] = otherEdge;
                    triLinks[t].valid[i] = true;
                    triLinks[otherEdge.triangle].links[otherEdge.start] = edges[idx];
                    triLinks[otherEdge.triangle].valid[otherEdge.start] = true;
                    break;
                }
            }
        }
    }

    // Reorder vertex rings
    // 1. Swap naked edge to first position
    // 2. Order manifold vertex rings as fans
    for (int v = 0; v < numVerts; v++) {
        int card = vertCardinality[v];
        if (card == 0) continue;
        int first = vertFirstEdge[v];

        int nakedIdx = -1;
        int numNaked = 0;
        for (int k = 0; k < card; k++) {
            const ConnEdge& e = edges[first + k];
            if (!triLinks[e.triangle].valid[e.start]) {
                nakedIdx = k;
                numNaked++;
            }
        }

        bool manifold = (numNaked < 2) && (card > 0);
        bool border = (numNaked == 1) && manifold;

        if (nakedIdx > 0)
            std::swap(edges[first], edges[first + nakedIdx]);

        if (manifold) {
            for (int i = 0; i < card - 1; i++) {
                const ConnEdge& cur = edges[first + i];
                int prevSlot = (cur.start + 2) % 3;
                if (triLinks[cur.triangle].valid[prevSlot]) {
                    int nextTri = triLinks[cur.triangle].links[prevSlot].triangle;
                    if (edges[first + i + 1].triangle != nextTri) {
                        bool found = false;
                        for (int j = i + 2; j < card; j++) {
                            if (edges[first + j].triangle == nextTri) {
                                std::swap(edges[first + i + 1], edges[first + j]);
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            manifold = false;
                            border = false;
                            break;
                        }
                    }
                }
            }
            if (manifold) {
                const ConnEdge& lastEdge = edges[first + card - 1];
                int lastPrevSlot = (lastEdge.start + 2) % 3;
                if (border) {
                    if (triLinks[lastEdge.triangle].valid[lastPrevSlot]) {
                        manifold = false;
                        border = false;
                    }
                } else {
                    if (!triLinks[lastEdge.triangle].valid[lastPrevSlot] ||
                        triLinks[lastEdge.triangle].links[lastPrevSlot].triangle != edges[first].triangle) {
                        manifold = false;
                    }
                }
            }
        }
    }

    // Iterate ALL edges in vertex order
    struct EdgeQuality {
        int triIdx;
        int edgeSlot;
        float quality;
        bool valid;
    };

    std::vector<EdgeQuality> allEdgeQualities(numEdges);
    const auto& vt = geometry.m_vertices;
    for (int e = 0; e < numEdges; e++) {
        allEdgeQualities[e].valid = false;
        allEdgeQualities[e].quality = FLT_MAX;
        allEdgeQualities[e].triIdx = -1;
        allEdgeQualities[e].edgeSlot = -1;

        const ConnEdge& edge = edges[e];
        int t = edge.triangle;
        int slot = edge.start;

        if (!triLinks[t].valid[slot]) continue;

        int vis[4];
        vis[0] = triVert(t, slot);
        vis[1] = triVert(t, (slot + 1) % 3);
        vis[2] = triVert(t, (slot + 2) % 3);
        vis[3] = triVert(triLinks[t].links[slot].triangle, (triLinks[t].links[slot].start + 2) % 3);

        hkAabb aabb;
        aabb.setEmpty();
        for (int i = 0; i < 4; i++) {
            hkVector4 p; p.set(vt[vis[i]][0], vt[vis[i]][1], vt[vis[i]][2]);
            aabb.includePoint(p);
        }

        float aabbSA = aabb.surfaceArea();
        if (aabbSA <= FLT_EPSILON) continue;

        auto calcTwiceSA = [&](int i0, int i1, int i2) -> float {
            float d0x = vt[i1][0]-vt[i0][0], d0y = vt[i1][1]-vt[i0][1], d0z = vt[i1][2]-vt[i0][2];
            float d1x = vt[i2][0]-vt[i0][0], d1y = vt[i2][1]-vt[i0][1], d1z = vt[i2][2]-vt[i0][2];
            float cx = d0y*d1z - d0z*d1y, cy = d0z*d1x - d0x*d1z, cz = d0x*d1y - d0y*d1x;
            return std::sqrt(cx*cx + cy*cy + cz*cz);
        };

        float quadSA = calcTwiceSA(vis[0], vis[1], vis[2]) + calcTwiceSA(vis[0], vis[1], vis[3]);
        float quality = (aabbSA - quadSA) / aabbSA;

        if (vis[0] < vis[1]) {
            allEdgeQualities[e].triIdx = t;
            allEdgeQualities[e].edgeSlot = slot;
        } else {
            allEdgeQualities[e].triIdx = triLinks[t].links[slot].triangle;
            allEdgeQualities[e].edgeSlot = triLinks[t].links[slot].start;
        }
        allEdgeQualities[e].quality = quality;
        allEdgeQualities[e].valid = true;
    }

    // Sort by quality (unstable)
    quickSort(allEdgeQualities.data(), (int)allEdgeQualities.size(),
        [](const EdgeQuality& a, const EdgeQuality& b) { return a.quality < b.quality; });

    struct TriPlane { float nx, ny, nz, w; };
    std::vector<TriPlane> triPlanes(numTris);
    for (int t = 0; t < numTris; t++) {
        float v0x = vt[triVert(t,0)][0], v0y = vt[triVert(t,0)][1], v0z = vt[triVert(t,0)][2];
        float v1x = vt[triVert(t,1)][0], v1y = vt[triVert(t,1)][1], v1z = vt[triVert(t,1)][2];
        float v2x = vt[triVert(t,2)][0], v2y = vt[triVert(t,2)][1], v2z = vt[triVert(t,2)][2];
        float e1x = v1x-v0x, e1y = v1y-v0y, e1z = v1z-v0z;
        float e2x = v2x-v0x, e2y = v2y-v0y, e2z = v2z-v0z;
        float nx = e1y*e2z - e1z*e2y;
        float ny = e1z*e2x - e1x*e2z;
        float nz = e1x*e2y - e1y*e2x;
        float lenSq = nx*nx + ny*ny + nz*nz;
        float invLen = (lenSq > 0.0f) ? (1.0f / sqrtf(lenSq)) : 0.0f;
        nx *= invLen; ny *= invLen; nz *= invLen;
        triPlanes[t].nx = nx; triPlanes[t].ny = ny; triPlanes[t].nz = nz;
        triPlanes[t].w = -(nx*v0x + ny*v0y + nz*v0z);
    }

    const float edgeClassifyTol = 0.01f;

    auto getApexVertex = [&](int adjTri, int adjEdgeStart) -> int {
        return triVert(adjTri, (adjEdgeStart + 2) % 3);
    };

    auto isEdgeConcaveOrFlat = [&](int tri, int edgeSlot) -> bool {
        auto& link = triLinks[tri].links[edgeSlot];
        if (link.triangle < 0) return false;
        int apexVert = getApexVertex(link.triangle, link.start);
        float ax = vt[apexVert][0], ay = vt[apexVert][1], az = vt[apexVert][2];
        float xy = triPlanes[tri].nx*ax + triPlanes[tri].ny*ay;
        float zw = triPlanes[tri].nz*az + triPlanes[tri].w;
        float d = xy + zw;
        if (d < -edgeClassifyTol) return false;
        return true;
    };

    auto isTriangleConcaveOrFlat = [&](int tri) -> bool {
        for (int e = 0; e < 3; e++) {
            if (!isEdgeConcaveOrFlat(tri, e)) return false;
        }
        return true;
    };

    std::vector<bool> triDisableAllEdges(numTris, false);

    // Greedy quad assignment with material and extent checks
    std::vector<bool> triUsed(numTris, false);

    for (auto& eq : allEdgeQualities) {
        if (!eq.valid) continue;

        int t = eq.triIdx;
        int slot = eq.edgeSlot;
        int ot = triLinks[t].links[slot].triangle;

        if (triUsed[t] || triUsed[ot]) continue;

        if (shapeTags[t] != shapeTags[ot]) continue;

        int vis[4];
        vis[0] = triVert(t, (slot + 1) % 3);
        vis[1] = triVert(t, (slot + 2) % 3);
        vis[2] = triVert(t, slot);
        vis[3] = triVert(ot, (triLinks[t].links[slot].start + 2) % 3);

        hkAabb aabb;
        aabb.setEmpty();
        for (int i = 0; i < 4; i++) {
            hkVector4 p; p.set(vt[vis[i]][0], vt[vis[i]][1], vt[vis[i]][2]);
            aabb.includePoint(p);
        }
        hkVector4 ext;
        aabb.getExtents(ext);
        if (ext[0] > maxExtent || ext[1] > maxExtent || ext[2] > maxExtent) continue;

        bool dae = false;
        if (isEdgeConcaveOrFlat(t, slot) && isTriangleConcaveOrFlat(t) && isTriangleConcaveOrFlat(ot))
            dae = true;
        triDisableAllEdges[t] = dae;
        triDisableAllEdges[ot] = dae;

        MeshPrimitive prim;
        prim.verts[0] = vis[0];
        prim.verts[1] = vis[1];
        prim.verts[2] = vis[2];
        prim.verts[3] = vis[3];
        prim.shapeTag = shapeTags[t];
        prim.isQuad = true;
        prim.disableAllEdges = dae;
        outPrimitives.push_back(prim);

        triUsed[t] = true;
        triUsed[ot] = true;
    }

    int numQuads = (int)outPrimitives.size();

    // Phase 1: Emit standalone triangles with initial flags (isTriangleConcaveOrFlat only)
    std::vector<int> standaloneTriOrder;
    for (int t = 0; t < numTris; t++) {
        if (triUsed[t]) continue;
        standaloneTriOrder.push_back(t);

        bool dae = isTriangleConcaveOrFlat(t);
        triDisableAllEdges[t] = dae;

        int bestEdge = 0;
        if (triVert(t, 0) < triVert(t, 1)) {
            bestEdge = 1;
            if (triVert(t, 1) < triVert(t, 2))
                bestEdge = 2;
        }

        MeshPrimitive prim;
        prim.verts[0] = triVert(t, bestEdge);
        prim.verts[1] = triVert(t, (bestEdge + 1) % 3);
        prim.verts[2] = triVert(t, (bestEdge + 2) % 3);
        prim.verts[3] = triVert(t, (bestEdge + 2) % 3);
        prim.shapeTag = shapeTags[t];
        prim.isQuad = false;
        prim.disableAllEdges = dae;
        outPrimitives.push_back(prim);
    }

    // Phase 2: Post process standalone triangles with canAllEdgesBeDisabled (cascading)
    int primBase = numQuads;
    for (int si = 0; si < (int)standaloneTriOrder.size(); si++) {
        int t = standaloneTriOrder[si];
        if (triDisableAllEdges[t]) continue;

        bool canDisable = true;
        for (int e = 0; e < 3; e++) {
            auto& link = triLinks[t].links[e];
            if (link.triangle < 0) { canDisable = false; break; }
            if (isEdgeConcaveOrFlat(link.triangle, link.start)) { canDisable = false; break; }
            if (triDisableAllEdges[link.triangle]) { canDisable = false; break; }
        }
        if (canDisable) {
            triDisableAllEdges[t] = true;
            outPrimitives[primBase + si].disableAllEdges = true;
        }
    }
}

struct BvhPoint {
    float pos[3];
    int index;
};

struct BvhNode {
    float lx[4], hx[4], ly[4], hy[4], lz[4], hz[4];
    hkUint32 data[4];
    bool isLeaf;
    bool isActive;

    void clear() {
        static constexpr uint32_t cMinBits = 0x7F7FFFEEu;
        static constexpr uint32_t cMaxBits = 0xFF7FFFEEu;
        float fMin, fMax;
        memcpy(&fMin, &cMinBits, 4);
        memcpy(&fMax, &cMaxBits, 4);
        for (int i = 0; i < 4; i++) {
            lx[i] = ly[i] = lz[i] = fMin;
            hx[i] = hy[i] = hz[i] = fMax;
            data[i] = 0;
        }
        isLeaf = false;
        isActive = false;
    }
    void clearLeaf() {
        clear();
        isLeaf = true;
        isActive = true;
        for (int i = 0; i < 4; i++) data[i] = 0xFFFFFFFF;
    }
    void clearInternal() {
        clear();
        isLeaf = false;
        isActive = true;
    }
    void setChild(int i, const hkAabb& childAabb, hkUint32 childData) {
        lx[i] = childAabb.m_min[0]; hx[i] = childAabb.m_max[0];
        ly[i] = childAabb.m_min[1]; hy[i] = childAabb.m_max[1];
        lz[i] = childAabb.m_min[2]; hz[i] = childAabb.m_max[2];
        data[i] = childData;
    }
    bool isDataValid(int i) const {
        if (isLeaf) return data[i] != 0xFFFFFFFF;
        return data[i] != 0;
    }
    void getAabb(int i, hkAabb& out) const {
        out.m_min.set(lx[i], ly[i], lz[i], 0);
        out.m_max.set(hx[i], hy[i], hz[i], 0);
    }
    void getCompoundAabb(hkAabb& out) const {
        out.setEmpty();
        for (int i = 0; i < 4; i++) {
            if (!isDataValid(i)) continue;
            hkAabb a;
            getAabb(i, a);
            out.includeAabb(a);
        }
    }
    int countActiveChildren() const {
        int c = 0;
        for (int i = 0; i < 4; i++) if (isDataValid(i)) c++;
        return c;
    }
};

struct BvhBuilder {
    std::vector<BvhNode> nodes;
    std::vector<hkAabb> aabbs;

    void build(BvhPoint* points, int numPoints, const hkAabb* primitiveAabbs) {
        aabbs.assign(primitiveAabbs, primitiveAabbs + numPoints);

        // Node 0 is unused, node 1 is root
        nodes.resize(2);
        nodes[0].clear();
        nodes[1].clear();

        if (numPoints == 0) return;

        buildHierarchy(points, 0, numPoints, 1);
        refit();
        sortByAabbsSize();
    }

    void buildHierarchy(BvhPoint* points, int start, int count, int nodeIndex) {
        struct StackEntry { int start, count, nodeIndex; };
        std::vector<StackEntry> stack;
        stack.push_back({start, count, nodeIndex});

        while (!stack.empty()) {
            StackEntry cur = stack.back();
            stack.pop_back();

            if (cur.count <= 32) {
                processSmallRange(points, cur.start, cur.count, cur.nodeIndex);
                continue;
            }

            int splits[5];
            splits[0] = cur.start;
            splits[4] = cur.start + cur.count;
            splitSah3(points, splits[0], splits[4], splits[2], 2);
            splitSah3(points, splits[0], splits[2], splits[1], 1);
            splitSah3(points, splits[2], splits[4], splits[3], 1);

            int subStart[4], subCount[4];
            int numSubs = 0;
            for (int i = 0; i < 4; i++) {
                int s = splits[i], e = splits[i + 1];
                if (e > s) {
                    subStart[numSubs] = s;
                    subCount[numSubs] = e - s;
                    numSubs++;
                }
            }

            nodes[cur.nodeIndex].clearInternal();
            int firstChild = (int)nodes.size();
            for (int i = 0; i < numSubs; i++)
                nodes.emplace_back();

            for (int i = 0; i < numSubs; i++) {
                int childIdx = firstChild + i;
                nodes[cur.nodeIndex].data[i] = childIdx;
                if (subCount[i] <= 4) {
                    nodes[childIdx].clearLeaf();
                    for (int j = 0; j < subCount[i]; j++)
                        nodes[childIdx].data[j] = points[subStart[i] + j].index;
                }
            }

            for (int i = 0; i < numSubs; i++) {
                if (subCount[i] > 4) {
                    stack.push_back({subStart[i], subCount[i], firstChild + i});
                }
            }
        }
    }

    // Insertion sort for small arrays
    template<typename Cmp>
    static void bvhInsertionSort(BvhPoint* arr, int size, Cmp cmpLess) {
        for (int i = 0; i < size; i++) {
            BvhPoint key = arr[i];
            int j = i;
            while (j > 0 && cmpLess(key, arr[j - 1])) {
                arr[j] = arr[j - 1];
                j--;
            }
            arr[j] = key;
        }
    }

    // Recursive SAH reordering with power-of-2 pivot
    void sahReorderSmallRange(BvhPoint* points, int start, int count) {
        if (count <= 4) return;

        int pow2 = 4;
        while (pow2 < count) pow2 *= 2;
        int pivot = pow2 / 2;

        // Try all 3 axes with fixed pivot, pick best
        float bestScore = FLT_MAX;
        int bestAxis = 0;
        std::vector<BvhPoint> axisSets[3];

        for (int axis = 0; axis < 3; axis++) {
            axisSets[axis].assign(points + start, points + start + count);
            bvhInsertionSort(axisSets[axis].data(), count, [axis](const BvhPoint& a, const BvhPoint& b) {
                return a.pos[axis] < b.pos[axis];
            });

            hkAabb left, right;
            left.setEmpty(); right.setEmpty();
            for (int i = 0; i < count; i++) {
                if (i < pivot) left.includeAabb(aabbs[axisSets[axis][i].index]);
                else right.includeAabb(aabbs[axisSets[axis][i].index]);
            }

            float score = left.surfaceArea() + right.surfaceArea();
            if (score < bestScore) {
                bestScore = score;
                bestAxis = axis;
            }
        }

        memcpy(points + start, axisSets[bestAxis].data(), count * sizeof(BvhPoint));

        sahReorderSmallRange(points, start, pivot);
        sahReorderSmallRange(points, start + pivot, count - pivot);
    }

    void processSmallRange(BvhPoint* points, int start, int count, int nodeIndex) {
        sahReorderSmallRange(points, start, count);

        int offset = start;
        int remaining = count;
        int currentRoot = nodeIndex;
        bool hasLeftOvers = true;

        while (hasLeftOvers) {
            int numSubRanges = 0;
            int subStarts[4], subSizes[4];

            while (remaining > 4 && numSubRanges < 3) {
                subStarts[numSubRanges] = offset;
                subSizes[numSubRanges] = 4;
                numSubRanges++;
                offset += 4;
                remaining -= 4;
            }
            if (remaining > 0) {
                subStarts[numSubRanges] = offset;
                subSizes[numSubRanges] = remaining;
                numSubRanges++;
            }

            nodes[currentRoot].clearInternal();
            hasLeftOvers = false;

            int firstChildIdx = (int)nodes.size();
            for (int i = 0; i < numSubRanges; i++)
                nodes.emplace_back();

            for (int i = 0; i < numSubRanges; i++) {
                int childIdx = firstChildIdx + i;
                nodes[currentRoot].data[i] = childIdx;

                if (subSizes[i] <= 4) {
                    nodes[childIdx].clearLeaf();
                    for (int j = 0; j < subSizes[i]; j++)
                        nodes[childIdx].data[j] = points[subStarts[i] + j].index;
                } else {
                    hasLeftOvers = true;
                    currentRoot = childIdx;
                    offset = subStarts[i];
                    remaining = subSizes[i];
                }
            }
        }
    }

    int findBestAxis(const BvhPoint* points, int start, int count) const {
        hkAabb bounds;
        bounds.setEmpty();
        for (int i = 0; i < count; i++) {
            hkVector4 p;
            p.set(points[start + i].pos[0], points[start + i].pos[1], points[start + i].pos[2]);
            bounds.includePoint(p);
        }
        hkVector4 ext;
        bounds.getExtents(ext);
        int bestAxis = 0;
        if (ext[1] > ext[bestAxis]) bestAxis = 1;
        if (ext[2] > ext[bestAxis]) bestAxis = 2;
        return bestAxis;
    }

    void splitSah3(BvhPoint* points, int start, int end, int& splitPos, int minCount = 1) {
        int count = end - start;
        if (count <= 1) { splitPos = end; return; }

        float bestCost = FLT_MAX;
        int bestAxis = 0;
        int bestSplit = start + count / 2;
        std::vector<BvhPoint> axisSets[3];
        float axisBestCost[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        int axisBestSplit[3] = {0, 0, 0};

        for (int axis = 0; axis < 3; axis++) {
            axisSets[axis].assign(points + start, points + end);
            quickSort(axisSets[axis].data(), count, [axis](const BvhPoint& a, const BvhPoint& b) {
                return a.pos[axis] < b.pos[axis];
            });

            std::vector<float> scores(count);
            hkAabb runningAabb;
            runningAabb.setEmpty();
            for (int i = 0; i < count; i++) {
                runningAabb.includeAabb(aabbs[axisSets[axis][i].index]);
                scores[i] = (i + 1) * runningAabb.surfaceArea();
            }

            runningAabb.setEmpty();
            for (int i = count - 1, j = 1; i > 0; --i, ++j) {
                runningAabb.includeAabb(aabbs[axisSets[axis][i].index]);
                float sum = scores[i - 1] + j * runningAabb.surfaceArea();
                if (sum < bestCost) {
                    bestCost = sum;
                    bestAxis = axis;
                    bestSplit = start + i;
                }
                if (sum < axisBestCost[axis]) {
                    axisBestCost[axis] = sum;
                    axisBestSplit[axis] = i;
                }
            }
        }

        memcpy(points + start, axisSets[bestAxis].data(), count * sizeof(BvhPoint));

        int leftSize = bestSplit - start;
        int rightSize = end - bestSplit;
        if (leftSize < minCount || rightSize < minCount) {
            bestSplit = start + count / 2;
        }

        splitPos = bestSplit;
    }

    void refit() {
        for (int i = (int)nodes.size() - 1; i >= 0; i--) {
            BvhNode& node = nodes[i];
            if (node.isLeaf) {
                for (int c = 0; c < 4; c++) {
                    if (node.data[c] == 0xFFFFFFFF) {
                        for (int a = 0; a < 3; a++) {
                            (&node.lx[0])[a * 8 + c] = 0;
                            (&node.hx[0])[a * 8 + c] = 0;
                        }
                        continue;
                    }
                    const hkAabb& a = aabbs[node.data[c]];
                    node.lx[c] = a.m_min[0]; node.hx[c] = a.m_max[0];
                    node.ly[c] = a.m_min[1]; node.hy[c] = a.m_max[1];
                    node.lz[c] = a.m_min[2]; node.hz[c] = a.m_max[2];
                }
            } else {
                for (int c = 0; c < 4; c++) {
                    if (node.data[c] == 0) {
                        node.lx[c] = node.hx[c] = 0;
                        node.ly[c] = node.hy[c] = 0;
                        node.lz[c] = node.hz[c] = 0;
                        continue;
                    }
                    hkAabb childAabb;
                    nodes[node.data[c]].getCompoundAabb(childAabb);
                    node.lx[c] = childAabb.m_min[0]; node.hx[c] = childAabb.m_max[0];
                    node.ly[c] = childAabb.m_min[1]; node.hy[c] = childAabb.m_max[1];
                    node.lz[c] = childAabb.m_min[2]; node.hz[c] = childAabb.m_max[2];
                }
            }
        }
    }

    void sortByAabbsSize() {
        struct SortEntry {
            float volume;
            int originalIndex;
            bool operator<(const SortEntry& rhs) const { return volume > rhs.volume; }
        };

        for (auto& node : nodes) {
            if (!node.isActive) continue;

            SortEntry entries[4];
            int activeCount = 0;
            hkUint32 savedData[4];
            float savedLx[4], savedHx[4], savedLy[4], savedHy[4], savedLz[4], savedHz[4];

            for (int i = 0; i < 4; i++) {
                savedData[i] = node.data[i];
                savedLx[i] = node.lx[i]; savedHx[i] = node.hx[i];
                savedLy[i] = node.ly[i]; savedHy[i] = node.hy[i];
                savedLz[i] = node.lz[i]; savedHz[i] = node.hz[i];

                if (node.isDataValid(i)) {
                    float ex = node.hx[i] - node.lx[i];
                    float ey = node.hy[i] - node.ly[i];
                    float ez = node.hz[i] - node.lz[i];
                    entries[activeCount].volume = ex * ey * ez;
                    entries[activeCount].originalIndex = i;
                    activeCount++;
                }
            }

            // Insertion sort
            for (int i = 1; i < activeCount; i++) {
                SortEntry key = entries[i];
                int j = i;
                while (j > 0 && key < entries[j - 1]) {
                    entries[j] = entries[j - 1];
                    j--;
                }
                entries[j] = key;
            }

            for (int i = 0; i < activeCount; i++) {
                int orig = entries[i].originalIndex;
                node.data[i] = savedData[orig];
                node.lx[i] = savedLx[orig]; node.hx[i] = savedHx[orig];
                node.ly[i] = savedLy[orig]; node.hy[i] = savedHy[orig];
                node.lz[i] = savedLz[orig]; node.hz[i] = savedHz[orig];
            }
            for (int i = activeCount; i < 4; i++) {
                node.data[i] = node.isLeaf ? 0xFFFFFFFF : 0;
                node.lx[i] = node.hx[i] = 0;
                node.ly[i] = node.hy[i] = 0;
                node.lz[i] = node.hz[i] = 0;
            }
        }
    }

    void toSimdTree(hkcdSimdTree::Node* outNodes, int& outCount) const {
        outCount = (int)nodes.size();
        for (int i = 0; i < outCount; i++) {
            const BvhNode& src = nodes[i];
            hkcdSimdTree::Node& dst = outNodes[i];
            memset(&dst, 0, sizeof(dst));
            for (int c = 0; c < 4; c++) {
                dst.m_lx[c] = src.lx[c]; dst.m_hx[c] = src.hx[c];
                dst.m_ly[c] = src.ly[c]; dst.m_hy[c] = src.hy[c];
                dst.m_lz[c] = src.lz[c]; dst.m_hz[c] = src.hz[c];
                dst.m_data[c] = src.data[c];
            }
            dst.m_isLeaf = src.isLeaf;
            dst.m_isActive = src.isActive;
        }
    }
};

static int countPrimitivesInSubTree(const std::vector<BvhNode>& nodes, int nodeIdx) {
    const BvhNode& node = nodes[nodeIdx];
    if (node.isLeaf) {
        int count = 0;
        for (int i = 0; i < 4; i++)
            if (node.data[i] != 0xFFFFFFFF) count++;
        return count;
    }
    int count = 0;
    for (int i = 0; i < 4; i++) {
        if (node.data[i] != 0) count += countPrimitivesInSubTree(nodes, node.data[i]);
    }
    return count;
}

struct VertexHash {
    float x, y, z;
    hkUint64 hash;
    bool operator<(const VertexHash& o) const { return hash < o.hash; }
};

static int countUniqueVerticesInSubTree(const std::vector<BvhNode>& nodes, int nodeIdx,
    const hkVector4* verts, const MeshPrimitive* prims) {
    std::vector<VertexHash> allVerts;

    std::vector<int> stack = {nodeIdx};
    while (!stack.empty()) {
        int idx = stack.back(); stack.pop_back();
        const BvhNode& n = nodes[idx];
        if (n.isLeaf) {
            for (int i = 0; i < 4; i++) {
                if (n.data[i] == 0xFFFFFFFF) continue;
                const auto& mp = prims[n.data[i]];
                int numV = mp.isQuad ? 4 : 3;
                for (int j = 0; j < numV; j++) {
                    VertexHash vh;
                    vh.x = verts[mp.verts[j]][0]; vh.y = verts[mp.verts[j]][1]; vh.z = verts[mp.verts[j]][2];
                    vh.hash = spatialHash(vh.x, vh.y, vh.z);
                    allVerts.push_back(vh);
                }
            }
        } else {
            for (int i = 0; i < 4; i++)
                if (n.data[i] != 0) stack.push_back(n.data[i]);
        }
    }

    quickSort(allVerts.data(), (int)allVerts.size(),
        [](const VertexHash& a, const VertexHash& b) { return a.hash < b.hash; });
    int unique = 0;
    for (int i = 0; i < (int)allVerts.size(); i++) {
        if (i > 0 && allVerts[i].hash == allVerts[i - 1].hash &&
            allVerts[i].x == allVerts[i - 1].x && allVerts[i].y == allVerts[i - 1].y && allVerts[i].z == allVerts[i - 1].z)
            continue;
        unique++;
    }
    return unique;
}

static bool canSubTreeFitIntoSection(const std::vector<BvhNode>& nodes, int nodeIdx, float maxAllowedExtent,
    const hkVector4* verts, const MeshPrimitive* prims) {
    hkAabb nodeAabb;
    nodes[nodeIdx].getCompoundAabb(nodeAabb);

    hkVector4 ext;
    nodeAabb.getExtents(ext);
    for (int i = 0; i < 3; i++)
        if (ext[i] > maxAllowedExtent) return false;

    int primCount = countPrimitivesInSubTree(nodes, nodeIdx);
    if ((int)(1.1f * primCount) > hknpMeshShape::GeometrySection::s_MaxVertexCountPerSection)
        return false;

    int uniqueVerts = countUniqueVerticesInSubTree(nodes, nodeIdx, verts, prims);
    if (uniqueVerts > hknpMeshShape::GeometrySection::s_MaxVertexCountPerSection)
        return false;
    return true;
}

static int findVertexInSection(const std::vector<float>& verts, float x, float y, float z) {
    int numVerts = (int)verts.size() / 3;
    for (int i = 0; i < numVerts; i++) {
        if (verts[i * 3 + 0] == x && verts[i * 3 + 1] == y && verts[i * 3 + 2] == z)
            return i;
    }
    return -1;
}

static void buildSectionGeometry(
    const std::vector<BvhNode>& bvhNodes, int subTreeRoot,
    const hkVector4* verts, const MeshPrimitive* prims,
    float maxVertexError, TempGeometrySection& section)
{
    bvhNodes[subTreeRoot].getCompoundAabb(section.originalDomain);
    section.domain = section.originalDomain;
    section.domain.expandBy(maxVertexError);
    section.domain.m_min[3] = 0; section.domain.m_max[3] = 0;

    Aabb8Quantizer converter(section.domain);
    for (int i = 0; i < 3; i++) {
        section.bitOffset[i] = (hkInt16)converter.bitOffset[i];
        ((float*)&section.bitScale8Inv)[i] = converter.bitScaleInv[i];
    }

    struct StackEntry { int bvhIdx; int parentSectionIdx; };
    std::vector<StackEntry> stack;
    stack.push_back({subTreeRoot, -1});
    std::vector<bool> nodeIsLeaf;

    while (!stack.empty()) {
        StackEntry entry = stack.back(); stack.pop_back();
        const BvhNode& node = bvhNodes[entry.bvhIdx];
        int currentNodeIndex = (int)section.sectionBvh.size();

        hknpAabb8TreeNode& aabb8Node = section.sectionBvh.emplace_back();
        memset(&aabb8Node, 0, sizeof(aabb8Node));
        nodeIsLeaf.push_back(node.isLeaf);

        for (int i = 0; i < 4; i++) {
            if (!node.isDataValid(i)) {
                clearAabb8(aabb8Node, i);
                continue;
            }
            hkAabb childAabb;
            node.getAabb(i, childAabb);
            for (int a = 0; a < 3; a++) {
                childAabb.m_min[a] = std::max(childAabb.m_min[a], section.domain.m_min[a]);
                childAabb.m_max[a] = std::min(childAabb.m_max[a], section.domain.m_max[a]);
            }
            hkUint8 lx, hx, ly, hy, lz, hz;
            converter.convertAabb(childAabb, &lx, &hx, &ly, &hy, &lz, &hz);
            setAabb8(aabb8Node, i, lx, hx, ly, hy, lz, hz);
        }

        if (entry.parentSectionIdx != -1) {
            hknpAabb8TreeNode& parent = section.sectionBvh[entry.parentSectionIdx];
            for (int i = 0; i < 4; i++) {
                if (parent.m_data[i] == 0) {
                    parent.m_data[i] = (hkUint8)currentNodeIndex;
                    break;
                }
            }
        }

        if (node.isLeaf) {
            for (int i = 0; i < 4; i++) {
                if (node.data[i] == 0xFFFFFFFF) continue;
                int primIdx = node.data[i];
                const auto& mp = prims[primIdx];
                section.shapeTags.push_back(mp.shapeTag);

                int numVerts = mp.isQuad ? 4 : 3;
                hknpMeshShape::GeometrySection::Primitive secPrim;
                hkUint8* dst = &secPrim.m_aId;

                for (int v = 0; v < numVerts; v++) {
                    float vx = verts[mp.verts[v]][0], vy = verts[mp.verts[v]][1], vz = verts[mp.verts[v]][2];
                    int vidx = findVertexInSection(section.verticesFlat, vx, vy, vz);
                    if (vidx == -1) {
                        vidx = (int)section.verticesFlat.size() / 3;
                        section.verticesFlat.push_back(vx);
                        section.verticesFlat.push_back(vy);
                        section.verticesFlat.push_back(vz);
                    }
                    dst[v] = (hkUint8)vidx;
                }
                if (!mp.isQuad) secPrim.m_dId = secPrim.m_cId;

                aabb8Node.m_data[i] = (hkUint8)section.primitives.size();
                section.primitives.push_back(secPrim);
                section.primitiveDisableAllEdges.push_back(mp.disableAllEdges);
            }
        } else {
            for (int i = 3; i >= 0; i--) {
                if (node.data[i] != 0) {
                    stack.push_back({(int)node.data[i], currentNodeIndex});
                }
            }
        }
    }

    for (int i = 0; i < (int)section.sectionBvh.size(); i++) {
        configureAsLeafOrInternal(section.sectionBvh[i], nodeIsLeaf[i]);
    }
}

static void buildSectionFromPrimitives(
    const int* primIndices, int numPrims,
    const hkVector4* verts, const MeshPrimitive* meshPrims,
    float maxVertexError, TempGeometrySection& section)
{
    hkAabb aabb;
    aabb.setEmpty();
    hkAabb primAabbs[4];

    for (int i = 0; i < numPrims; i++) {
        const auto& mp = meshPrims[primIndices[i]];
        int numVerts = mp.isQuad ? 4 : 3;
        primAabbs[i].setEmpty();

        int startVertexIndex = (int)section.verticesFlat.size() / 3;
        for (int v = 0; v < numVerts; v++) {
            float vx = verts[mp.verts[v]][0], vy = verts[mp.verts[v]][1], vz = verts[mp.verts[v]][2];
            hkVector4 p; p.set(vx, vy, vz);
            primAabbs[i].includePoint(p);
            section.verticesFlat.push_back(vx);
            section.verticesFlat.push_back(vy);
            section.verticesFlat.push_back(vz);
        }

        hknpMeshShape::GeometrySection::Primitive prim;
        prim.m_aId = (hkUint8)(startVertexIndex + 0);
        prim.m_bId = (hkUint8)(startVertexIndex + 1);
        prim.m_cId = (hkUint8)(startVertexIndex + 2);
        prim.m_dId = (hkUint8)(startVertexIndex + 3);
        if (!mp.isQuad) prim.m_dId = prim.m_cId;

        section.primitives.push_back(prim);
        section.shapeTags.push_back(mp.shapeTag);
        section.primitiveDisableAllEdges.push_back(mp.disableAllEdges);

        aabb.includeAabb(primAabbs[i]);
    }

    section.originalDomain = aabb;
    section.domain = aabb;
    section.domain.expandBy(maxVertexError);

    Aabb8Quantizer converter(section.domain);
    for (int i = 0; i < 3; i++) {
        section.bitOffset[i] = (hkInt16)converter.bitOffset[i];
        ((float*)&section.bitScale8Inv)[i] = converter.bitScaleInv[i];
    }

    hknpAabb8TreeNode& leaf = section.sectionBvh.emplace_back();
    memset(leaf.m_data, 0, sizeof(leaf.m_data));
    for (int i = 0; i < 4; i++) clearAabb8(leaf, i);

    for (int i = 0; i < numPrims; i++) {
        hkUint8 lx, hx, ly, hy, lz, hz;
        converter.convertAabb(primAabbs[i], &lx, &hx, &ly, &hy, &lz, &hz);
        setAabb8(leaf, i, lx, hx, ly, hy, lz, hz);
        leaf.m_data[i] = (hkUint8)i;
    }

    configureAsLeafOrInternal(leaf, true);
}

static void quantizeSection(const hknpMeshShapeVertexConversionUtil& util, TempGeometrySection& section) {
    getSectionOffset(util, section.originalDomain, section.sectionOffset);

    int numVerts = (int)section.verticesFlat.size() / 3;
    section.quantizedVertices.resize(numVerts + 1); // +1 for safe unpacking
    memset(&section.quantizedVertices[numVerts], 0, sizeof(hknpMeshShape::GeometrySection::Vertex16_3));

    for (int i = 0; i < numVerts; i++) {
        packVertex(util, &section.verticesFlat[i * 3], section.sectionOffset, &section.quantizedVertices[i].m_x);
    }
}

static void refitSectionBvh(const hknpMeshShapeVertexConversionUtil& util, TempGeometrySection& section) {
    Aabb8Quantizer converter(section.domain);

    for (int nodeIdx = (int)section.sectionBvh.size() - 1; nodeIdx >= 0; nodeIdx--) {
        hknpAabb8TreeNode& node = section.sectionBvh[nodeIdx];

        if (isAabb8TreeNodeLeaf(node)) {
            for (int c = 0; c < 4; c++) {
                if (!isAabb8TreeNodeChildValid(node, c)) continue;

                int primIdx = getAabb8TreeNodePrimitiveIndex(node, c);
                const auto& prim = section.primitives[primIdx];
                hkUint8 ids[4] = {prim.m_aId, prim.m_bId, prim.m_cId, prim.m_dId};
                int numVerts = (prim.m_cId == prim.m_dId) ? 3 : 4;

                hkAabb primAabb;
                primAabb.setEmpty();
                for (int v = 0; v < numVerts; v++) {
                    float unpacked[3];
                    unpackVertex(util, &section.quantizedVertices[ids[v]].m_x, section.sectionOffset, unpacked);
                    hkVector4 p; p.set(unpacked[0], unpacked[1], unpacked[2]);
                    primAabb.includePoint(p);
                }

                hkUint8 lx, hx, ly, hy, lz, hz;
                converter.convertAabb(primAabb, &lx, &hx, &ly, &hy, &lz, &hz);
                setAabb8(node, c, lx, hx, ly, hy, lz, hz);
            }
        } else {
            for (int c = 0; c < 4; c++) {
                if (!isAabb8TreeNodeChildValid(node, c)) continue;
                int childIdx = node.m_data[c];
                hkUint8 clx, chx, cly, chy, clz, chz;
                compoundAabb8(section.sectionBvh[childIdx], clx, chx, cly, chy, clz, chz);
                setAabb8(node, c, clx, chx, cly, chy, clz, chz);
            }
        }
    }
}

static bool checkFlatConvexQuad(const float* a, const float* b, const float* c, const float* d) {
    float vAB[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
    float vAC[3] = {c[0]-a[0], c[1]-a[1], c[2]-a[2]};
    float nx = vAB[1]*vAC[2] - vAB[2]*vAC[1];
    float ny = vAB[2]*vAC[0] - vAB[0]*vAC[2];
    float nz = vAB[0]*vAC[1] - vAB[1]*vAC[0];
    float nLenSq = nx*nx + ny*ny + nz*nz;

    float p30[3] = {a[0]-d[0], a[1]-d[1], a[2]-d[2]};
    float outOfPlane = std::abs(p30[0]*nx + p30[1]*ny + p30[2]*nz);
    float tol = 1e-3f;
    if (outOfPlane * outOfPlane > tol * tol * nLenSq)
        return false;

    float vAD[3] = {d[0]-a[0], d[1]-a[1], d[2]-a[2]};
    float n2x = vAC[1]*vAD[2] - vAC[2]*vAD[1];
    float n2y = vAC[2]*vAD[0] - vAC[0]*vAD[2];
    float n2z = vAC[0]*vAD[1] - vAC[1]*vAD[0];
    float n2LenSq = n2x*n2x + n2y*n2y + n2z*n2z;
    if (n2LenSq < nLenSq * 0.01f)
        return false;

    float vCD[3] = {d[0]-c[0], d[1]-c[1], d[2]-c[2]};
    float vCB[3] = {b[0]-c[0], b[1]-c[1], b[2]-c[2]};
    float cx0[3], cx1[3];
    cx0[0] = vAB[1]*vAD[2] - vAB[2]*vAD[1];
    cx0[1] = vAB[2]*vAD[0] - vAB[0]*vAD[2];
    cx0[2] = vAB[0]*vAD[1] - vAB[1]*vAD[0];
    cx1[0] = vCD[1]*vCB[2] - vCD[2]*vCB[1];
    cx1[1] = vCD[2]*vCB[0] - vCD[0]*vCB[2];
    cx1[2] = vCD[0]*vCB[1] - vCD[1]*vCB[0];
    float dot = cx0[0]*cx1[0] + cx0[1]*cx1[1] + cx0[2]*cx1[2];
    return dot >= 0.0f;
}

static bool areTrianglesEquivalent(hkUint8 a0, hkUint8 a1, hkUint8 a2,
                                    hkUint8 b0, hkUint8 b1, hkUint8 b2, int& offsetOut) {
    for (int o = 0; o < 3; o++) {
        hkUint8 r[3] = {b0, b1, b2};
        if (a0 == r[o % 3] && a1 == r[(o+1) % 3] && a2 == r[(o+2) % 3]) {
            offsetOut = o;
            return true;
        }
    }
    return false;
}

static void setIsFlatConvexQuad(hknpMeshShape::GeometrySection::Primitive& p, bool isFlatConvex) {
    bool current = p.m_bId > p.m_dId;
    if (current == isFlatConvex) return;

    hkUint8 firstTri[3] = {p.m_aId, p.m_cId, p.m_dId};
    for (int offset = 0; offset <= 2; offset++) {
        hkUint8 result[4];
        for (int i = 0; i < 3; i++)
            result[i] = firstTri[(i + offset) % 3];
        result[3] = p.m_bId;

        if ((result[1] > result[3]) != isFlatConvex)
            continue;

        int otherOffset;
        if (!areTrianglesEquivalent(result[0], result[2], result[3],
                                     p.m_aId, p.m_bId, p.m_cId, otherOffset))
            continue;

        p.m_aId = result[0];
        p.m_bId = result[1];
        p.m_cId = result[2];
        p.m_dId = result[3];
        break;
    }
}

static void markFlatConvexQuads(TempGeometrySection& section) {
    for (int i = 0; i < (int)section.primitives.size(); i++) {
        auto& prim = section.primitives[i];
        if (prim.m_cId == prim.m_dId) continue;
        setIsFlatConvexQuad(prim, false);
    }
}

static void flagInteriorTriangles(TempGeometrySection& section) {
    int numPrims = (int)section.primitives.size();
    int numBytes = (numPrims + 7) / 8;
    section.interiorBitField.resize(numBytes, 0);

    int count = 0;
    for (int p = 0; p < numPrims; p++) {
        if (p < (int)section.primitiveDisableAllEdges.size() && section.primitiveDisableAllEdges[p]) {
            section.interiorBitField[p / 8] |= (1 << (p % 8));
            count++;
        }
    }
}

static int bitWidth(int v) {
    if (v <= 0) return 0;
    int bits = 0;
    while ((1 << bits) <= v) bits++;
    return bits;
}

static int computeSectionShift(int numSections, int maxPrimsPerSection) {
    int numPrimBits = bitWidth(maxPrimsPerSection - 1);
    return numPrimBits + 1;
}

static int computeSectionShiftFromKeyBits(int numShapeKeyBits, int numSections) {
    int numSectionBits = bitWidth(numSections - 1);
    return numShapeKeyBits - numSectionBits;
}

static hkUint32 packMeshPrimitiveKey(int sectionIndex, int primitiveIndex, int subTriangle, int sectionShift) {
    return ((hkUint32)sectionIndex << sectionShift) | ((hkUint32)primitiveIndex << 1) | (hkUint32)subTriangle;
}

static std::vector<hknpMeshShape::ShapeTagTableEntry> buildShapeTagTable(const std::vector<TempGeometrySection>& sections, int sectionShift) {
    std::vector<hknpMeshShape::ShapeTagTableEntry> result;

    hknpMeshShape::ShapeTagTableEntry entry;
    entry.m_meshPrimitiveKey = 0;
    entry.m_shapeTag = !sections.empty() ? sections[0].shapeTags[0] : 0xFFFF;
    entry._pad = 0;
    result.push_back(entry);

    for (int s = 0; s < (int)sections.size(); s++) {
        for (int p = 0; p < (int)sections[s].shapeTags.size(); p++) {
            hkUint16 tag = sections[s].shapeTags[p];
            if (result.back().m_shapeTag != tag) {
                entry.m_meshPrimitiveKey = packMeshPrimitiveKey(s, p, 0, sectionShift);
                entry.m_shapeTag = tag;
                entry._pad = 0;
                result.push_back(entry);
            }
        }
    }

    entry.m_meshPrimitiveKey = packMeshPrimitiveKey((int)sections.size(), 0xFF, 0, sectionShift);
    entry.m_shapeTag = 0xFFFF;
    entry._pad = 0;
    result.push_back(entry);

    return result;
}

static void reorderSectionByShapeTag(TempGeometrySection& section) {
    int numPrims = (int)section.primitives.size();
    if (numPrims <= 1) return;

    std::vector<int> perm(numPrims);
    std::iota(perm.begin(), perm.end(), 0);
    quickSort(perm.data(), numPrims, [&](int a, int b) {
        return section.shapeTags[a] < section.shapeTags[b];
    });

    // Check if already sorted
    bool sorted = true;
    for (int i = 0; i < numPrims; i++) {
        if (perm[i] != i) { sorted = false; break; }
    }
    if (sorted) return;

    // Apply permutation
    std::vector<hknpMeshShape::GeometrySection::Primitive> newPrims(numPrims);
    std::vector<hkUint16> newTags(numPrims);
    std::vector<bool> newDAE(numPrims, false);
    for (int i = 0; i < numPrims; i++) {
        newPrims[i] = section.primitives[perm[i]];
        newTags[i] = section.shapeTags[perm[i]];
        if (perm[i] < (int)section.primitiveDisableAllEdges.size())
            newDAE[i] = section.primitiveDisableAllEdges[perm[i]];
    }
    section.primitives = newPrims;
    section.shapeTags = newTags;
    section.primitiveDisableAllEdges = newDAE;

    // Rebuild BVH leaf data mapping
    std::vector<int> inversePerm(numPrims);
    for (int i = 0; i < numPrims; i++) inversePerm[perm[i]] = i;

    for (auto& node : section.sectionBvh) {
        if (!isAabb8TreeNodeLeaf(node)) continue;
        for (int c = 0; c < 4; c++) {
            if (!isAabb8TreeNodeChildValid(node, c)) continue;
            int oldIdx = getAabb8TreeNodePrimitiveIndex(node, c);
            node.m_data[c] = (hkUint8)inversePerm[oldIdx];
        }
        configureAsLeafOrInternal(node, true);
    }

    // Rebuild interior bitfield
    int numBytes = (numPrims + 7) / 8;
    std::vector<hkUint8> newBitField(numBytes, 0);
    for (int i = 0; i < numPrims; i++) {
        int oldIdx = perm[i];
        if (section.interiorBitField[oldIdx / 8] & (1 << (oldIdx % 8)))
            newBitField[i / 8] |= (1 << (i % 8));
    }
    section.interiorBitField = newBitField;
}

hknpMeshShape* MeshShapeBuilder::build(const hkGeometry& geometry, float maxVertexError) {
    int numTriangles = geometry.m_numTriangles;
    if (numTriangles == 0) return nullptr;

    hknpMeshShapeVertexConversionUtil vertexConvUtil;
    initVertexConversionUtil(vertexConvUtil, maxVertexError);
    float maxAllowedExtent = calcMaxAllowedSectionExtent(maxVertexError);

    // Copy geometry for modification (welding modifies it)
    hkGeometry geom;
    geom.m_numVertices = geometry.m_numVertices;
    geom.m_vertices = new hkVector4[geometry.m_numVertices];
    memcpy(geom.m_vertices, geometry.m_vertices, geometry.m_numVertices * sizeof(hkVector4));
    geom.m_numTriangles = geometry.m_numTriangles;
    geom.m_triangles = new hkGeometry::Triangle[geometry.m_numTriangles];
    memcpy(geom.m_triangles, geometry.m_triangles, geometry.m_numTriangles * sizeof(hkGeometry::Triangle));

    // Weld duplicate vertices
    weldDuplicateVertices(geom);

    // Create shape tags
    std::vector<hkUint16> shapeTags(geom.m_numTriangles);
    for (int i = 0; i < geom.m_numTriangles; i++)
        shapeTags[i] = geom.m_triangles[i].m_material;

    // Create quad-dominant geometry
    std::vector<MeshPrimitive> meshPrimitives;
    createQuadDominantGeometry(geom, shapeTags, maxAllowedExtent, meshPrimitives);

    // Reorder: triangles first, then quads
    std::stable_partition(meshPrimitives.begin(), meshPrimitives.end(),
        [](const MeshPrimitive& p) { return !p.isQuad; });
    int numPrimitives = (int)meshPrimitives.size();

    // Build per-primitive AABBs and points for BVH
    std::vector<hkAabb> primAabbs(numPrimitives);
    std::vector<BvhPoint> points(numPrimitives);

    for (int i = 0; i < numPrimitives; i++) {
        const auto& mp = meshPrimitives[i];
        hkAabb& aabb = primAabbs[i];
        aabb.setEmpty();
        int numV = mp.isQuad ? 4 : 3;
        for (int v = 0; v < numV; v++) {
            hkVector4 p; p.set(geom.m_vertices[mp.verts[v]][0], geom.m_vertices[mp.verts[v]][1], geom.m_vertices[mp.verts[v]][2]);
            aabb.includePoint(p);
        }
        hkVector4 center;
        aabb.getCenter(center);
        points[i].pos[0] = center[0];
        points[i].pos[1] = center[1];
        points[i].pos[2] = center[2];
        points[i].index = i;
    }

    // Build BVH
    BvhBuilder bvh;
    bvh.build(points.data(), numPrimitives, primAabbs.data());

    // Traverse BVH and split into sections
    std::vector<TempGeometrySection> sections;
    std::vector<int> bvhStack = {1}; // root

    struct SectionTreeEntry {
        BvhNode node;
    };
    std::vector<SectionTreeEntry> sectionTreeNodes;
    sectionTreeNodes.push_back({});
    sectionTreeNodes[0].node.clear();

    struct StkItem { int parentIdx; int childSlot; };
    std::vector<StkItem> sectionTreeStack = {{0, 0}};

    while (!bvhStack.empty()) {
        int subTreeRoot = bvhStack.back(); bvhStack.pop_back();
        const BvhNode& rootNode = bvh.nodes[subTreeRoot];

        bool convertToSection[4] = {};
        int validChildren = 0;

        // iterate 3->0 for stack push so children pop in order 0->3
        for (int i = 3; i >= 0; i--) {
            if (!rootNode.isDataValid(i)) continue;
            validChildren++;

            if (rootNode.isLeaf) {
                convertToSection[i] = true;
            } else {
                convertToSection[i] = canSubTreeFitIntoSection(bvh.nodes, rootNode.data[i], maxAllowedExtent,
                    geom.m_vertices, meshPrimitives.data());
            }

            if (!convertToSection[i]) {
                bvhStack.push_back(rootNode.data[i]);
            }
        }

        int numInSection = 0;
        std::vector<int> sectionIndices;

        if (!rootNode.isLeaf) {
            for (int i = 0; i < validChildren; i++) {
                if (!convertToSection[i]) continue;
                TempGeometrySection sec;
                buildSectionGeometry(bvh.nodes, rootNode.data[i], geom.m_vertices, meshPrimitives.data(),
                    maxVertexError, sec);
                sections.push_back(std::move(sec));
                sectionIndices.push_back((int)sections.size() - 1);
                numInSection++;
            }
        } else {
            // Leaf node: group primitives into sections
            int primIndices[4];
            int primCount = 0;
            for (int i = 0; i < 4; i++) {
                if (rootNode.data[i] == 0xFFFFFFFF) continue;
                primIndices[primCount++] = rootNode.data[i];
            }

            // Recompute primitive AABBs from geometry
            hkAabb leafAabbs[4];
            for (int i = 0; i < primCount; i++) {
                const auto& mp = meshPrimitives[primIndices[i]];
                int numV = mp.isQuad ? 4 : 3;
                hkVector4 v[4];
                for (int j = 0; j < numV; j++)
                    v[j].set(geom.m_vertices[mp.verts[j]][0], geom.m_vertices[mp.verts[j]][1], geom.m_vertices[mp.verts[j]][2]);
                if (numV == 3) v[3] = v[2];
                leafAabbs[i].setFromTetrahedron(v[0], v[1], v[2], v[3]);
            }

            bool used[4] = {};
            int groupBuf[4];
            int groupSize = 0;

            groupBuf[0] = primIndices[0];
            groupSize = 1;
            used[0] = true;
            int remaining = primCount - 1;
            hkAabb groupAabb = leafAabbs[0];
            int numGroups = 0;

            while (remaining > 0) {
                bool expanded = false;
                for (int i = 0; i < primCount; i++) {
                    if (used[i]) continue;
                    hkAabb newAabb = groupAabb;
                    newAabb.includeAabb(leafAabbs[i]);
                    hkVector4 ext;
                    newAabb.getExtents(ext);
                    if (ext[0] <= maxAllowedExtent && ext[1] <= maxAllowedExtent && ext[2] <= maxAllowedExtent) {
                        groupBuf[groupSize++] = primIndices[i];
                        used[i] = true;
                        expanded = true;
                        groupAabb = newAabb;
                        remaining--;
                    }
                }
                if (!expanded && remaining > 0) {
                    TempGeometrySection sec;
                    buildSectionFromPrimitives(groupBuf, groupSize, geom.m_vertices, meshPrimitives.data(),
                        maxVertexError, sec);
                    sections.push_back(std::move(sec));
                    sectionIndices.push_back((int)sections.size() - 1);
                    numGroups++;

                    for (int i = 0; i < primCount; i++) {
                        if (!used[i]) {
                            groupBuf[0] = primIndices[i];
                            groupSize = 1;
                            used[i] = true;
                            remaining--;
                            break;
                        }
                    }
                }
            }

            // Emit final group
            TempGeometrySection sec;
            buildSectionFromPrimitives(groupBuf, groupSize, geom.m_vertices, meshPrimitives.data(),
                maxVertexError, sec);
            sections.push_back(std::move(sec));
            sectionIndices.push_back((int)sections.size() - 1);
            numInSection = primCount;
        }

        // Update section tree
        {
            bool allConverted = (validChildren == numInSection);
            int newNodeIdx = (int)sectionTreeNodes.size();
            sectionTreeNodes.push_back({});

            if (allConverted)
                sectionTreeNodes[newNodeIdx].node.clearLeaf();
            else
                sectionTreeNodes[newNodeIdx].node.clearInternal();

            if (!sectionTreeStack.empty()) {
                auto item = sectionTreeStack.back(); sectionTreeStack.pop_back();
                if (item.parentIdx != 0)
                    sectionTreeNodes[item.parentIdx].node.data[item.childSlot] = newNodeIdx;
            }

            if (numInSection == 0 || numInSection == validChildren) {
                if (!rootNode.isLeaf) {
                    // Copy AABBs from BVH node
                    for (int i = 0; i < 4; i++) {
                        if (rootNode.isDataValid(i)) {
                            hkAabb a; rootNode.getAabb(i, a);
                            sectionTreeNodes[newNodeIdx].node.lx[i] = a.m_min[0]; sectionTreeNodes[newNodeIdx].node.hx[i] = a.m_max[0];
                            sectionTreeNodes[newNodeIdx].node.ly[i] = a.m_min[1]; sectionTreeNodes[newNodeIdx].node.hy[i] = a.m_max[1];
                            sectionTreeNodes[newNodeIdx].node.lz[i] = a.m_min[2]; sectionTreeNodes[newNodeIdx].node.hz[i] = a.m_max[2];
                        }
                    }
                    sectionTreeNodes[newNodeIdx].node.isLeaf = allConverted;

                    for (int i = validChildren - 1; i >= 0; i--) {
                        if (allConverted) {
                            sectionTreeNodes[newNodeIdx].node.data[i] = sectionIndices[i];
                        } else {
                            sectionTreeStack.push_back({newNodeIdx, i});
                        }
                    }
                } else {
                    // Leaf BVH node with groups
                    sectionTreeNodes[newNodeIdx].node.clearLeaf();
                    for (int i = 0; i < (int)sectionIndices.size(); i++) {
                        sectionTreeNodes[newNodeIdx].node.setChild(i, sections[sectionIndices[i]].originalDomain, sectionIndices[i]);
                    }
                }
            } else {
                // Mixed: 0 < numInSection < validChildren
                // Create extra leaf node for the converted sections
                int leafNodeIdx = (int)sectionTreeNodes.size();
                sectionTreeNodes.push_back({});
                sectionTreeNodes[leafNodeIdx].node.clearLeaf();

                for (int i = 0; i < (int)sectionIndices.size(); i++) {
                    sectionTreeNodes[leafNodeIdx].node.setChild(i, sections[sectionIndices[i]].originalDomain, sectionIndices[i]);
                }

                // Find first converted child slot to place the leaf node
                int leafChildIdx = -1;
                for (int i = 0; i < validChildren; i++) {
                    if (convertToSection[i]) {
                        hkAabb leafAabb;
                        sectionTreeNodes[leafNodeIdx].node.getCompoundAabb(leafAabb);
                        sectionTreeNodes[newNodeIdx].node.setChild(i, leafAabb, leafNodeIdx);
                        leafChildIdx = i;
                        break;
                    }
                }

                // Place non-converted children, skipping leafChildIdx slot
                int slot = validChildren - (int)sectionIndices.size();
                for (int i = validChildren - 1; i >= 0; i--) {
                    if (!convertToSection[i]) {
                        if (slot == leafChildIdx) slot--;
                        hkAabb a; rootNode.getAabb(i, a);
                        sectionTreeNodes[newNodeIdx].node.lx[slot] = a.m_min[0]; sectionTreeNodes[newNodeIdx].node.hx[slot] = a.m_max[0];
                        sectionTreeNodes[newNodeIdx].node.ly[slot] = a.m_min[1]; sectionTreeNodes[newNodeIdx].node.hy[slot] = a.m_max[1];
                        sectionTreeNodes[newNodeIdx].node.lz[slot] = a.m_min[2]; sectionTreeNodes[newNodeIdx].node.hz[slot] = a.m_max[2];
                        sectionTreeStack.push_back({newNodeIdx, slot});
                        slot--;
                    }
                }
            }
        }
    }

    // Quantize, refit, flag interior, reorder
    for (int si = 0; si < (int)sections.size(); si++) {
        auto& sec = sections[si];
        quantizeSection(vertexConvUtil, sec);
        markFlatConvexQuads(sec);
        refitSectionBvh(vertexConvUtil, sec);

        // Compute refit domain from dequantized primitive vertices
        sec.refitDomain.setEmpty();
        for (int p = 0; p < (int)sec.primitives.size(); p++) {
            const auto& prim = sec.primitives[p];
            hkUint8 ids[4] = {prim.m_aId, prim.m_bId, prim.m_cId, prim.m_dId};
            int nv = (prim.m_cId == prim.m_dId) ? 3 : 4;
            for (int v = 0; v < nv; v++) {
                float u[3];
                unpackVertex(vertexConvUtil, &sec.quantizedVertices[ids[v]].m_x, sec.sectionOffset, u);
                hkVector4 pt; pt.set(u[0], u[1], u[2]);
                sec.refitDomain.includePoint(pt);
            }
        }

        flagInteriorTriangles(sec);
        reorderSectionByShapeTag(sec);
    }
    // Compute section shift for shape key encoding
    int maxPrimsPerSection = 0;
    for (auto& sec : sections)
        maxPrimsPerSection = std::max(maxPrimsPerSection, (int)sec.primitives.size());
    int sectionShift = computeSectionShift((int)sections.size(), maxPrimsPerSection);

    // Build shape tag table
    auto shapeTagTable = buildShapeTagTable(sections, sectionShift);

    // Build section tree (top-level BVH over sections)
    int numSectionTreeNodes = (int)sectionTreeNodes.size();
    std::vector<hkcdSimdTree::Node> finalSectionTree(numSectionTreeNodes);
    for (int i = 0; i < numSectionTreeNodes; i++) {
        auto& src = sectionTreeNodes[i].node;
        auto& dst = finalSectionTree[i];
        memset(&dst, 0, sizeof(dst));
        for (int c = 0; c < 4; c++) {
            dst.m_lx[c] = src.lx[c]; dst.m_hx[c] = src.hx[c];
            dst.m_ly[c] = src.ly[c]; dst.m_hy[c] = src.hy[c];
            dst.m_lz[c] = src.lz[c]; dst.m_hz[c] = src.hz[c];
            dst.m_data[c] = src.data[c];
        }
        dst.m_isLeaf = src.isLeaf;
        dst.m_isActive = false;
    }

    // Refit section tree from dequantized section domains
    for (int i = numSectionTreeNodes - 1; i >= 0; i--) {
        auto& node = finalSectionTree[i];
        if (node.m_isLeaf) {
            for (int c = 0; c < 4; c++) {
                if (node.m_data[c] == 0xFFFFFFFF || (node.m_data[c] == 0 && !node.m_isLeaf)) continue;
                int secIdx = node.m_data[c];
                if (secIdx >= 0 && secIdx < (int)sections.size()) {
                    const auto& sec = sections[secIdx];
                    node.m_lx[c] = sec.refitDomain.m_min[0]; node.m_hx[c] = sec.refitDomain.m_max[0];
                    node.m_ly[c] = sec.refitDomain.m_min[1]; node.m_hy[c] = sec.refitDomain.m_max[1];
                    node.m_lz[c] = sec.refitDomain.m_min[2]; node.m_hz[c] = sec.refitDomain.m_max[2];
                }
            }
        } else {
            for (int c = 0; c < 4; c++) {
                if (node.m_data[c] == 0) continue;
                int childIdx = node.m_data[c];
                if (childIdx > 0 && childIdx < numSectionTreeNodes) {
                    hkAabb childAabb;
                    childAabb.setEmpty();
                    auto& childNode = finalSectionTree[childIdx];
                    for (int cc = 0; cc < 4; cc++) {
                        bool valid = childNode.m_isLeaf ? (childNode.m_data[cc] != 0xFFFFFFFF) : (childNode.m_data[cc] != 0);
                        if (!valid) continue;
                        hkAabb a;
                        a.m_min.set(childNode.m_lx[cc], childNode.m_ly[cc], childNode.m_lz[cc]);
                        a.m_max.set(childNode.m_hx[cc], childNode.m_hy[cc], childNode.m_hz[cc]);
                        childAabb.includeAabb(a);
                    }
                    node.m_lx[c] = childAabb.m_min[0]; node.m_hx[c] = childAabb.m_max[0];
                    node.m_ly[c] = childAabb.m_min[1]; node.m_hy[c] = childAabb.m_max[1];
                    node.m_lz[c] = childAabb.m_min[2]; node.m_hz[c] = childAabb.m_max[2];
                }
            }
        }
    }

    // Sort section tree children by AABB volume (descending)
    for (int i = 0; i < numSectionTreeNodes; i++) {
        auto& node = finalSectionTree[i];

        struct ChildSort {
            float volume;
            int slot;
            bool operator<(const ChildSort& rhs) const { return volume > rhs.volume; }
        };
        ChildSort entries[4];
        int active = 0;

        float slx[4], shx[4], sly[4], shy[4], slz[4], shz[4];
        hkUint32 sdata[4];
        for (int c = 0; c < 4; c++) {
            slx[c] = node.m_lx[c]; shx[c] = node.m_hx[c];
            sly[c] = node.m_ly[c]; shy[c] = node.m_hy[c];
            slz[c] = node.m_lz[c]; shz[c] = node.m_hz[c];
            sdata[c] = node.m_data[c];
            bool valid = node.m_isLeaf ? (node.m_data[c] != 0xFFFFFFFF) : (node.m_data[c] != 0);
            if (valid) {
                float ex = node.m_hx[c] - node.m_lx[c];
                float ey = node.m_hy[c] - node.m_ly[c];
                float ez = node.m_hz[c] - node.m_lz[c];
                entries[active].volume = ex * ey * ez;
                entries[active].slot = c;
                active++;
            }
        }

        for (int a = 1; a < active; a++) {
            ChildSort key = entries[a];
            int j = a;
            while (j > 0 && key < entries[j - 1]) {
                entries[j] = entries[j - 1];
                j--;
            }
            entries[j] = key;
        }

        for (int a = 0; a < active; a++) {
            int orig = entries[a].slot;
            node.m_lx[a] = slx[orig]; node.m_hx[a] = shx[orig];
            node.m_ly[a] = sly[orig]; node.m_hy[a] = shy[orig];
            node.m_lz[a] = slz[orig]; node.m_hz[a] = shz[orig];
            node.m_data[a] = sdata[orig];
        }
        for (int a = active; a < 4; a++) {
            static constexpr uint32_t cMinBits = 0x7F7FFFEEu;
            static constexpr uint32_t cMaxBits = 0xFF7FFFEEu;
            float cMin, cMax;
            memcpy(&cMin, &cMinBits, 4);
            memcpy(&cMax, &cMaxBits, 4);
            node.m_lx[a] = cMin; node.m_hx[a] = cMax;
            node.m_ly[a] = cMin; node.m_hy[a] = cMax;
            node.m_lz[a] = cMin; node.m_hz[a] = cMax;
            node.m_data[a] = node.m_isLeaf ? 0xFFFFFFFF : 0;
        }
    }

    int numSections = (int)sections.size();

    // Calculate total size
    size_t shapeSize = ALIGN_UP(sizeof(hknpMeshShape), 16);
    size_t tagTableSize = ALIGN_UP(shapeTagTable.size() * sizeof(hknpMeshShape::ShapeTagTableEntry), 16);
    size_t treeNodesSize = ALIGN_UP(numSectionTreeNodes * sizeof(hkcdSimdTree::Node), 16);
    size_t sectionHeadersSize = ALIGN_UP(numSections * sizeof(hknpMeshShape::GeometrySection), 16);

    size_t totalSectionDataSize = 0;
    for (const auto& sec : sections) {
        totalSectionDataSize += ALIGN_UP(sec.sectionBvh.size() * sizeof(hknpAabb8TreeNode), 16);
        totalSectionDataSize += ALIGN_UP(sec.primitives.size() * sizeof(hknpMeshShape::GeometrySection::Primitive), 16);
        totalSectionDataSize += ALIGN_UP((sec.quantizedVertices.size() - 1) * sizeof(hknpMeshShape::GeometrySection::Vertex16_3), 16);
        totalSectionDataSize += ALIGN_UP(sec.interiorBitField.size(), 16);
    }

    size_t totalSize = shapeSize + tagTableSize + treeNodesSize + sectionHeadersSize + totalSectionDataSize;
    u8* buffer = new u8[totalSize];
    memset(buffer, 0, totalSize);

    hknpMeshShape* shape = reinterpret_cast<hknpMeshShape*>(buffer);

    // Fill shape header
    shape->mType = 8; 
    shape->mDispatchType = 3;
    shape->mFlags = 4;
    int numSectionBits = bitWidth((int)sections.size() - 1);
    int numPrimBits = bitWidth(maxPrimsPerSection - 1);
    shape->m_numShapeKeyBits = (hkUint8)(numSectionBits + numPrimBits + 1);
    shape->m_convexRadius = 0;
    shape->m_userData = 0;
    shape->m_shapeTagCodecInfo = 0xFFFFFFFF;
    shape->m_vertexConversionUtil = vertexConvUtil;

    // Properties
    shape->m_properties.m_properties.m_offset = 0;
    shape->m_properties.m_properties.m_size = 0;
    shape->m_properties.m_properties.m_capacityAndFlags = 0;

    // Fill shape tag table
    size_t tagTableOffset = shapeSize;
    hknpMeshShape::ShapeTagTableEntry* tagTablePtr = reinterpret_cast<hknpMeshShape::ShapeTagTableEntry*>(buffer + tagTableOffset);
    memcpy(tagTablePtr, shapeTagTable.data(), shapeTagTable.size() * sizeof(hknpMeshShape::ShapeTagTableEntry));
    shape->m_shapeTagTable.m_offset = (int)((u8*)tagTablePtr - (u8*)&shape->m_shapeTagTable);
    shape->m_shapeTagTable.m_size = (int)shapeTagTable.size();

    // Fill top level tree
    size_t treeOffset = tagTableOffset + tagTableSize;
    hkcdSimdTree::Node* treeNodesPtr = reinterpret_cast<hkcdSimdTree::Node*>(buffer + treeOffset);
    memcpy(treeNodesPtr, finalSectionTree.data(), numSectionTreeNodes * sizeof(hkcdSimdTree::Node));
    shape->m_topLevelTree.m_nodes.m_offset = (hkInt64)((u8*)treeNodesPtr - (u8*)&shape->m_topLevelTree.m_nodes);
    shape->m_topLevelTree.m_nodes.m_size = numSectionTreeNodes;
    shape->m_topLevelTree.m_nodes.m_capacityAndFlags = 0;
    shape->m_topLevelTree.m_isCompact = true;

    // Fill geometry sections
    size_t sectionsOffset = treeOffset + treeNodesSize;
    hknpMeshShape::GeometrySection* sectionsPtr = reinterpret_cast<hknpMeshShape::GeometrySection*>(buffer + sectionsOffset);
    shape->m_geometrySections.m_offset = (int)((u8*)sectionsPtr - (u8*)&shape->m_geometrySections);
    shape->m_geometrySections.m_size = numSections;

    // Fill section data
    size_t dataOffset = sectionsOffset + sectionHeadersSize;
    for (int s = 0; s < numSections; s++) {
        const auto& sec = sections[s];
        hknpMeshShape::GeometrySection& gs = sectionsPtr[s];

        // Section BVH
        size_t bvhSize = ALIGN_UP(sec.sectionBvh.size() * sizeof(hknpAabb8TreeNode), 16);
        hknpAabb8TreeNode* bvhPtr = reinterpret_cast<hknpAabb8TreeNode*>(buffer + dataOffset);
        memcpy(bvhPtr, sec.sectionBvh.data(), sec.sectionBvh.size() * sizeof(hknpAabb8TreeNode));
        gs.m_sectionBvh.m_offset = (int)((u8*)bvhPtr - (u8*)&gs.m_sectionBvh);
        gs.m_sectionBvh.m_size = (int)sec.sectionBvh.size();
        dataOffset += bvhSize;

        // Primitives
        size_t primSize = ALIGN_UP(sec.primitives.size() * sizeof(hknpMeshShape::GeometrySection::Primitive), 16);
        auto* primPtr = reinterpret_cast<hknpMeshShape::GeometrySection::Primitive*>(buffer + dataOffset);
        memcpy(primPtr, sec.primitives.data(), sec.primitives.size() * sizeof(hknpMeshShape::GeometrySection::Primitive));
        gs.m_primitives.m_offset = (int)((u8*)primPtr - (u8*)&gs.m_primitives);
        gs.m_primitives.m_size = (int)sec.primitives.size();
        dataOffset += primSize;

        // Vertex buffer
        int actualVertCount = (int)sec.quantizedVertices.size() - 1;
        size_t vertSize = ALIGN_UP(actualVertCount * sizeof(hknpMeshShape::GeometrySection::Vertex16_3), 16);
        auto* vertPtr = reinterpret_cast<hknpMeshShape::GeometrySection::Vertex16_3*>(buffer + dataOffset);
        memcpy(vertPtr, sec.quantizedVertices.data(), actualVertCount * sizeof(hknpMeshShape::GeometrySection::Vertex16_3));
        gs.m_vertexBuffer.m_offset = (int)((u8*)vertPtr - (u8*)&gs.m_vertexBuffer);
        gs.m_vertexBuffer.m_size = (int)sec.quantizedVertices.size() - 1;
        dataOffset += vertSize;

        // Interior bitfield
        size_t bitSize = ALIGN_UP(sec.interiorBitField.size(), 16);
        u8* bitPtr = buffer + dataOffset;
        memcpy(bitPtr, sec.interiorBitField.data(), sec.interiorBitField.size());
        gs.m_interiorPrimitiveBitField.m_offset = (int)(bitPtr - (u8*)&gs.m_interiorPrimitiveBitField);
        gs.m_interiorPrimitiveBitField.m_size = (int)sec.interiorBitField.size();
        dataOffset += bitSize;

        // Section metadata
        memcpy(gs.m_sectionOffset, sec.sectionOffset, sizeof(sec.sectionOffset));
        gs.m_bitScale8Inv = sec.bitScale8Inv;
        memcpy(gs.m_bitOffset, sec.bitOffset, sizeof(sec.bitOffset));
    }

    // No primitive mapping
    shape->m_primitiveMapping.m_offset = 0;

    delete[] geom.m_vertices;
    delete[] geom.m_triangles;

    return shape;
}

void MeshShapeBuilder::buildSurfaceGeometry(const hknpMeshShape* shape, hkGeometry& geometryOut) {
    std::vector<hkVector4> vertices;
    std::vector<hkGeometry::Triangle> triangles;

    const auto& vcu = shape->m_vertexConversionUtil;
    int numSections = shape->m_geometrySections.m_size;
    const auto* sections = shape->m_geometrySections.begin();

    int numTagEntries = shape->m_shapeTagTable.m_size;
    const auto* tagTable = shape->m_shapeTagTable.begin();

    int sectionShift = computeSectionShiftFromKeyBits(shape->m_numShapeKeyBits, numSections);

    int verticesOutIndex = 0;

    for (int s = 0; s < numSections; s++) {
        const auto& sec = sections[s];
        const auto* quantVerts = sec.m_vertexBuffer.begin();
        int numPrims = sec.m_primitives.m_size;
        const auto* prims = sec.m_primitives.begin();

        for (int p = 0; p < numPrims; p++) {
            const auto& prim = prims[p];

            hkVector4 primVerts[4];
            float ua[3], ub[3], uc[3], ud[3];
            unpackVertex(vcu, &quantVerts[prim.m_aId].m_x, sec.m_sectionOffset, ua);
            unpackVertex(vcu, &quantVerts[prim.m_bId].m_x, sec.m_sectionOffset, ub);
            unpackVertex(vcu, &quantVerts[prim.m_cId].m_x, sec.m_sectionOffset, uc);
            unpackVertex(vcu, &quantVerts[prim.m_dId].m_x, sec.m_sectionOffset, ud);
            primVerts[0].set(ua[0], ua[1], ua[2], 0);
            primVerts[1].set(ub[0], ub[1], ub[2], 0);
            primVerts[2].set(uc[0], uc[1], uc[2], 0);
            primVerts[3].set(ud[0], ud[1], ud[2], 0);

            bool isQuad = (prim.m_cId != prim.m_dId);
            int numTriangles = isQuad ? 2 : 1;

            hkUint32 meshPrimKey = packMeshPrimitiveKey(s, p, 0, sectionShift);
            int lo = 0, hi = numTagEntries - 1;
            while (lo < hi) {
                int mid = (lo + hi + 1) / 2;
                if (tagTable[mid].m_meshPrimitiveKey <= meshPrimKey) lo = mid;
                else hi = mid - 1;
            }
            int material = tagTable[lo].m_shapeTag;

            for (int t = 0; t < numTriangles; t++) {
                vertices.push_back(primVerts[0]);
                vertices.push_back(primVerts[1 + t]);
                vertices.push_back(primVerts[2 + t]);

                hkGeometry::Triangle tri;
                tri.m_a = verticesOutIndex;
                tri.m_b = verticesOutIndex + 1;
                tri.m_c = verticesOutIndex + 2;
                tri.m_material = material;
                triangles.push_back(tri);
                verticesOutIndex += 3;
            }
        }
    }

    geometryOut.m_numVertices = (int)vertices.size();
    geometryOut.m_vertices = new hkVector4[vertices.size()];
    memcpy(geometryOut.m_vertices, vertices.data(), vertices.size() * sizeof(hkVector4));

    geometryOut.m_numTriangles = (int)triangles.size();
    geometryOut.m_triangles = new hkGeometry::Triangle[triangles.size()];
    memcpy(geometryOut.m_triangles, triangles.data(), triangles.size() * sizeof(hkGeometry::Triangle));
}

}
