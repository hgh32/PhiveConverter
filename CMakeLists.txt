cmake_minimum_required(VERSION 3.14)
project(PhiveConverter CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)

add_executable(PhiveConverter
    source/main.cpp
    source/fivex/CollisionInfo.cpp
    source/fivex/PhiveWriter.cpp
    source/fivex/MeshShapeBuilder.cpp
    source/fivex/ClothInfo.cpp
    source/fivex/NavMeshInfo.cpp
    source/fivex/NvtInfo.cpp
    source/havok/hkTagfile.cpp
    source/fivex/AampFile.cpp
)

target_include_directories(PhiveConverter PRIVATE source)
target_link_libraries(PhiveConverter PRIVATE yaml-cpp)

# Copy hash dictionary next to executable
configure_file(${CMAKE_SOURCE_DIR}/aamp_hashes.txt ${CMAKE_BINARY_DIR}/aamp_hashes.txt COPYONLY)
