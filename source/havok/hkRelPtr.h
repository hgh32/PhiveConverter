#pragma once

#include "havok/hkTypes.h"

template<class T>
struct hkRelPtr {
    T* get() {
        if (m_offset == 0) return nullptr;
        return reinterpret_cast<T*>(reinterpret_cast<hkUint8*>(this) + m_offset);
    };
    const T* get() const {
        if (m_offset == 0) return nullptr;
        return reinterpret_cast<const T*>(reinterpret_cast<const hkUint8*>(this) + m_offset);
    };
    hkInt64 m_offset;
};
