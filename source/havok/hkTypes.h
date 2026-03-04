#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <algorithm>

typedef unsigned char hkUint8;
typedef unsigned short hkUint16;
typedef unsigned int hkUint32;
typedef unsigned long long hkUlong;
typedef unsigned long long hkUint64;
typedef char hkInt8;
typedef short hkInt16;
typedef short hkHalf16;
typedef int hkInt32;
typedef long long hkLong;
typedef long long hkInt64;
typedef float hkReal;
typedef bool hkBool;

struct hkVector4 {
    float v[4];
    float& operator[](int i) { return v[i]; }
    const float& operator[](int i) const { return v[i]; }
    float operator()(int i) const { return v[i]; }
    void setZero() { v[0] = v[1] = v[2] = v[3] = 0; }
    void set(float x, float y, float z, float w = 0) { v[0] = x; v[1] = y; v[2] = z; v[3] = w; }

    float getX() const { return v[0]; }
    float getY() const { return v[1]; }
    float getZ() const { return v[2]; }
    float getW() const { return v[3]; }

    bool equalXYZ(const hkVector4& o) const {
        return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2];
    }
    bool equalXYZ(const float* o) const {
        return v[0] == o[0] && v[1] == o[1] && v[2] == o[2];
    }
};
static_assert(sizeof(hkVector4) == 0x10);

struct hkFloat3 {
    float m_x;
    float m_y;
    float m_z;

    bool operator==(const hkFloat3& o) const {
        return m_x == o.m_x && m_y == o.m_y && m_z == o.m_z;
    }
};
static_assert(sizeof(hkFloat3) == 0xC);

struct hkAabb {
    hkVector4 m_min;
    hkVector4 m_max;

    void setEmpty() {
        m_min.set(FLT_MAX, FLT_MAX, FLT_MAX, 0);
        m_max.set(-FLT_MAX, -FLT_MAX, -FLT_MAX, 0);
    }
    bool isValid() const {
        return m_min[0] <= m_max[0] && m_min[1] <= m_max[1] && m_min[2] <= m_max[2];
    }
    void includePoint(const hkVector4& p) {
        for (int i = 0; i < 3; i++) {
            m_min[i] = std::min(m_min[i], p[i]);
            m_max[i] = std::max(m_max[i], p[i]);
        }
    }
    void includeAabb(const hkAabb& o) {
        for (int i = 0; i < 3; i++) {
            m_min[i] = std::min(m_min[i], o.m_min[i]);
            m_max[i] = std::max(m_max[i], o.m_max[i]);
        }
    }
    void setFromTetrahedron(const hkVector4& a, const hkVector4& b, const hkVector4& c, const hkVector4& d) {
        for (int i = 0; i < 3; i++) {
            m_min[i] = std::min(std::min(a[i], b[i]), std::min(c[i], d[i]));
            m_max[i] = std::max(std::max(a[i], b[i]), std::max(c[i], d[i]));
        }
        m_min[3] = m_max[3] = 0;
    }
    void expandBy(float e) {
        for (int i = 0; i < 3; i++) {
            m_min[i] -= e;
            m_max[i] += e;
        }
    }
    void getCenter(hkVector4& c) const {
        for (int i = 0; i < 4; i++)
            c[i] = (m_min[i] + m_max[i]) * 0.5f;
    }
    void getExtents(hkVector4& e) const {
        for (int i = 0; i < 3; i++)
            e[i] = m_max[i] - m_min[i];
        e[3] = 0;
    }
    float surfaceArea() const {
        float ex = m_max[0] - m_min[0];
        float ey = m_max[1] - m_min[1];
        float ez = m_max[2] - m_min[2];
        float p0 = (2.0f * ex) * ey;
        float p1 = (2.0f * ey) * ez;
        float p2 = (2.0f * ez) * ex;
        return p2 + (p1 + p0);
    }
    bool contains(const hkAabb& o) const {
        return o.m_min[0] >= m_min[0] && o.m_min[1] >= m_min[1] && o.m_min[2] >= m_min[2] &&
               o.m_max[0] <= m_max[0] && o.m_max[1] <= m_max[1] && o.m_max[2] <= m_max[2];
    }
};

struct hkGeometry {
    struct Triangle {
        int m_a;
        int m_b;
        int m_c;
        int m_material;
    };
    static_assert(sizeof(Triangle) == 0x10);

    hkVector4* m_vertices = nullptr;
    int m_numVertices = 0;
    Triangle* m_triangles = nullptr;
    int m_numTriangles = 0;

    ~hkGeometry() {}
};
