#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include "fivex/CollisionInfo.hpp"
#include "fivex/ClothInfo.hpp"
#include "fivex/NavMeshInfo.hpp"
#include "fivex/NvtInfo.hpp"
#include "fivex/AampFile.hpp"

using json = nlohmann::json;

bool endsWith(const std::string& str, const std::string& suffix) {
    return str.size() >= suffix.size() &&
           str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool loadPhiveConfig() {
    std::ifstream file("fxconfig.json");
    if (!file) {
        std::cerr << "Failed to read phive config file!\n";
        return false;
    }
    std::string configData((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    json config = json::parse(configData, nullptr, false);
    if (config.is_discarded()) {
        std::cerr << "Failed to parse phive config file!\n";
        return false;
    }
    FiveX::CollisionInfo::setConfig(config);
    return true;
}

bool convertPhiveToObj(const std::string& phivePath, const std::string& outDir) {
    std::ifstream inFile(phivePath, std::ios::binary | std::ios::ate);
    if (!inFile) {
        std::cerr << "Failed to open phive file: " << phivePath << "\n";
        return false;
    }
    std::streamsize size = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    std::vector<u8> bphsh((size_t)size);
    if (!inFile.read(reinterpret_cast<char*>(bphsh.data()), size)) {
        std::cerr << "Failed to read phive file: " << phivePath << "\n";
        return false;
    }

    FiveX::CollisionInfo info;
    info.loadFromBphsh(bphsh);

    std::vector<u8> outObj, outMatInfos;
    info.serializeToObj(outObj, outMatInfos);

    std::filesystem::path outPath(outDir);
    std::string outName = std::filesystem::path(phivePath).stem().string();
    std::string objPath = (outPath / (outName + ".obj")).string();
    std::string matInfoPath = (outPath / (outName + ".json")).string();

    std::ofstream objFile(objPath, std::ios::binary);
    if (!objFile) { std::cerr << "Failed to create obj file: " << objPath << "\n"; return false; }
    objFile.write(reinterpret_cast<const char*>(outObj.data()), outObj.size());

    std::ofstream matFile(matInfoPath, std::ios::binary);
    if (!matFile) { std::cerr << "Failed to create mat info file: " << matInfoPath << "\n"; return false; }
    matFile.write(reinterpret_cast<const char*>(outMatInfos.data()), outMatInfos.size());

    std::cout << "Saved obj and mats to " << std::filesystem::absolute(outDir) << "\n";
    return true;
}

bool convertBphclToYaml(const std::string& bphclPath, const std::string& outDir) {
    std::ifstream inFile(bphclPath, std::ios::binary | std::ios::ate);
    if (!inFile) { std::cerr << "Failed to open bphcl file: " << bphclPath << "\n"; return false; }
    std::streamsize size = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    std::vector<u8> data((size_t)size);
    if (!inFile.read(reinterpret_cast<char*>(data.data()), size)) {
        std::cerr << "Failed to read bphcl file: " << bphclPath << "\n"; return false;
    }

    FiveX::ClothInfo info;
    info.loadFromBphcl(data);

    std::vector<u8> tagYaml, aampYaml;
    info.serializeToYaml(tagYaml, aampYaml);

    std::filesystem::path outPath(outDir);
    std::string outName = std::filesystem::path(bphclPath).stem().string();
    std::string tagPath = (outPath / (outName + ".bphcl.yaml")).string();
    std::string aampPath = (outPath / (outName + ".bphcl.aamp.yaml")).string();

    std::ofstream tagFile(tagPath, std::ios::binary);
    if (!tagFile) { std::cerr << "Failed to create yaml file: " << tagPath << "\n"; return false; }
    tagFile.write(reinterpret_cast<const char*>(tagYaml.data()), tagYaml.size());

    if (!aampYaml.empty()) {
        std::ofstream aampFile(aampPath, std::ios::binary);
        if (!aampFile) { std::cerr << "Failed to create aamp yaml: " << aampPath << "\n"; return false; }
        aampFile.write(reinterpret_cast<const char*>(aampYaml.data()), aampYaml.size());
        std::cout << "Saved bphcl yaml to " << std::filesystem::absolute(tagPath)
                  << " + " << std::filesystem::absolute(aampPath) << "\n";
    } else {
        std::cout << "Saved bphcl yaml to " << std::filesystem::absolute(tagPath) << "\n";
    }
    return true;
}

bool convertYamlToBphcl(const std::string& yamlPath, const std::string& outPath) {
    std::ifstream tagFile(yamlPath);
    if (!tagFile) { std::cerr << "Failed to open yaml file: " << yamlPath << "\n"; return false; }
    std::string tagStr((std::istreambuf_iterator<char>(tagFile)), std::istreambuf_iterator<char>());
    std::vector<u8> tagYaml(tagStr.begin(), tagStr.end());

    // Try to load companion AAMP yaml
    std::string aampPath = yamlPath.substr(0, yamlPath.size() - 5) + ".aamp.yaml";
    std::vector<u8> aampYaml;
    std::ifstream aampFile(aampPath);
    if (aampFile) {
        std::string aampStr((std::istreambuf_iterator<char>(aampFile)), std::istreambuf_iterator<char>());
        aampYaml.assign(aampStr.begin(), aampStr.end());
    }

    FiveX::ClothInfo info;
    info.loadFromYaml(tagYaml, aampYaml);

    std::vector<u8> outData;
    info.serializeToBphcl(outData);

    std::ofstream outFile(outPath, std::ios::binary);
    if (!outFile) { std::cerr << "Failed to create output file: " << outPath << "\n"; return false; }
    outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());

    std::cout << "Saved bphcl to " << std::filesystem::absolute(outPath) << "\n";
    return true;
}

bool convertBphnmToYaml(const std::string& bphnmPath, const std::string& outDir) {
    std::ifstream inFile(bphnmPath, std::ios::binary | std::ios::ate);
    if (!inFile) { std::cerr << "Failed to open bphnm file: " << bphnmPath << "\n"; return false; }
    std::streamsize size = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    std::vector<u8> data((size_t)size);
    if (!inFile.read(reinterpret_cast<char*>(data.data()), size)) {
        std::cerr << "Failed to read bphnm file: " << bphnmPath << "\n"; return false;
    }

    FiveX::NavMeshInfo info;
    info.loadFromBphnm(data);

    std::vector<u8> yamlOut;
    info.serializeToYaml(yamlOut);

    std::filesystem::path outPath(outDir);
    std::string outName = std::filesystem::path(bphnmPath).stem().string();
    std::string yamlPath = (outPath / (outName + ".bphnm.yaml")).string();

    std::ofstream yamlFile(yamlPath, std::ios::binary);
    if (!yamlFile) { std::cerr << "Failed to create yaml file: " << yamlPath << "\n"; return false; }
    yamlFile.write(reinterpret_cast<const char*>(yamlOut.data()), yamlOut.size());

    std::cout << "Saved bphnm yaml to " << std::filesystem::absolute(yamlPath) << "\n";

    // Also export navmesh as OBJ for visualization
    std::vector<u8> objOut;
    if (info.serializeToObj(objOut)) {
        std::string objPath = (outPath / (outName + ".bphnm.obj")).string();
        std::ofstream objFile(objPath, std::ios::binary);
        if (objFile) {
            objFile.write(reinterpret_cast<const char*>(objOut.data()), objOut.size());
            std::cout << "Saved navmesh obj to " << std::filesystem::absolute(objPath) << "\n";
        }
    }

    return true;
}

bool convertYamlToBphnm(const std::string& yamlPath, const std::string& outPath) {
    std::ifstream yamlFile(yamlPath);
    if (!yamlFile) { std::cerr << "Failed to open yaml file: " << yamlPath << "\n"; return false; }
    std::string yamlStr((std::istreambuf_iterator<char>(yamlFile)), std::istreambuf_iterator<char>());
    std::vector<u8> yamlData(yamlStr.begin(), yamlStr.end());

    FiveX::NavMeshInfo info;
    info.loadFromYaml(yamlData);

    std::vector<u8> outData;
    info.serializeToBphnm(outData);

    std::ofstream outFile(outPath, std::ios::binary);
    if (!outFile) { std::cerr << "Failed to create output file: " << outPath << "\n"; return false; }
    outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());

    std::cout << "Saved bphnm to " << std::filesystem::absolute(outPath) << "\n";
    return true;
}

bool convertNvtToYaml(const std::string& nvtPath, const std::string& outDir) {
    std::ifstream inFile(nvtPath, std::ios::binary | std::ios::ate);
    if (!inFile) { std::cerr << "Failed to open nvt file: " << nvtPath << "\n"; return false; }
    std::streamsize size = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    std::vector<u8> data((size_t)size);
    if (!inFile.read(reinterpret_cast<char*>(data.data()), size)) {
        std::cerr << "Failed to read nvt file: " << nvtPath << "\n"; return false;
    }

    FiveX::NvtInfo info;
    info.loadFromNvt(data);

    std::vector<u8> yamlOut;
    info.serializeToYaml(yamlOut);

    std::filesystem::path outPath(outDir);
    std::string outName = std::filesystem::path(nvtPath).stem().string();
    std::string yamlPath = (outPath / (outName + ".nvt.yaml")).string();

    std::ofstream yamlFile(yamlPath, std::ios::binary);
    if (!yamlFile) { std::cerr << "Failed to create yaml file: " << yamlPath << "\n"; return false; }
    yamlFile.write(reinterpret_cast<const char*>(yamlOut.data()), yamlOut.size());
    std::cout << "Saved nvt yaml to " << std::filesystem::absolute(yamlPath) << "\n";

    // Also export OBJ with path waypoints
    std::vector<u8> objOut;
    info.serializeToObj(objOut);
    std::string objPath = (outPath / (outName + ".nvt.obj")).string();
    std::ofstream objFile(objPath, std::ios::binary);
    if (objFile) {
        objFile.write(reinterpret_cast<const char*>(objOut.data()), objOut.size());
        std::cout << "Saved nvt obj to " << std::filesystem::absolute(objPath) << "\n";
    }

    return true;
}

bool convertYamlToNvt(const std::string& yamlPath, const std::string& outPath) {
    std::ifstream yamlFile(yamlPath);
    if (!yamlFile) { std::cerr << "Failed to open yaml file: " << yamlPath << "\n"; return false; }
    std::string yamlStr((std::istreambuf_iterator<char>(yamlFile)), std::istreambuf_iterator<char>());
    std::vector<u8> yamlData(yamlStr.begin(), yamlStr.end());

    FiveX::NvtInfo info;
    info.loadFromYaml(yamlData);

    std::vector<u8> outData;
    info.serializeToNvt(outData);

    std::ofstream outFile(outPath, std::ios::binary);
    if (!outFile) { std::cerr << "Failed to create output file: " << outPath << "\n"; return false; }
    outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());

    std::cout << "Saved nvt to " << std::filesystem::absolute(outPath) << "\n";
    return true;
}

bool convertObjToPhive(const std::string& objPath, const std::string& matInfoPath, const std::string& outPath) {
    std::ifstream objFile(objPath);
    if (!objFile) { std::cerr << "Failed to open obj file: " << objPath << "\n"; return false; }
    std::string objData((std::istreambuf_iterator<char>(objFile)), std::istreambuf_iterator<char>());

    json matConfig;
    if (!matInfoPath.empty()) {
        std::ifstream matFile(matInfoPath);
        if (!matFile) { std::cerr << "Failed to open mat info file: " << matInfoPath << "\n"; return false; }
        std::string matData((std::istreambuf_iterator<char>(matFile)), std::istreambuf_iterator<char>());
        matConfig = json::parse(matData, nullptr, false);
        if (matConfig.is_discarded()) { std::cerr << "Failed to parse mat info json: " << matInfoPath << "\n"; return false; }
    }

    FiveX::CollisionInfo info;
    info.initialize((u8*)(objData.c_str()), objData.length(), matConfig);

    std::vector<u8> outData;
    info.serializeToBphsh(outData);

    std::ofstream outFile(outPath, std::ios::binary);
    if (!outFile) { std::cerr << "Failed to create output file: " << outPath << "\n"; return false; }
    outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());

    std::cout << "Saved bphsh to " << std::filesystem::absolute(outPath) << "\n";
    return true;
}

int main(int argc, const char** argv) {
    // Auto-load hash dictionary from next to executable
    {
        std::filesystem::path dictPath = std::filesystem::current_path() / "aamp_hashes.txt";
        if (std::filesystem::exists(dictPath)) {
            FiveX::AampFile::loadHashDictionary(dictPath.string());
        }
    }

    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "help") {
        std::cout << "Usage:\n"
                  << "  PhiveConverter FILE.bphsh                - convert bphsh to obj+json\n"
                  << "  PhiveConverter FILE.obj FILE.json        - convert obj+json to bphsh\n"
                  << "  PhiveConverter FILE.obj                  - convert obj to bphsh (no mat info)\n"
                  << "  PhiveConverter FILE.bphcl                - convert bphcl to yaml\n"
                  << "  PhiveConverter FILE.bphcl.yaml           - convert yaml to bphcl\n"
                  << "  PhiveConverter FILE.bphnm                - convert bphnm to yaml+obj\n"
                  << "  PhiveConverter FILE.bphnm.yaml           - convert yaml to bphnm\n"
                  << "  PhiveConverter FILE.bphnm.obj            - convert obj+yaml to bphnm (vertex edit)\n"
                  << "  PhiveConverter FILE.nvt                  - convert nvt to yaml+obj\n"
                  << "  PhiveConverter FILE.nvt.yaml             - convert yaml to nvt\n"
                  << "  PhiveConverter FILE.nvt.obj              - convert obj+yaml to nvt (waypoint edit)\n"
                  << "  PhiveConverter -p PHIVE -o OUTDIR        - phive to obj\n"
                  << "  PhiveConverter -obj OBJ -mat MAT -o OUT  - obj to phive\n";
        return 0;
    }

    // Simple file detection for new formats
    std::string firstArg = argv[1];
    if (firstArg[0] != '-') {
        // YAML format (primary)
        if (endsWith(firstArg, ".bphcl")) {
            std::string outDir = std::filesystem::path(firstArg).parent_path().string();
            if (outDir.empty()) outDir = ".";
            return convertBphclToYaml(firstArg, outDir) ? 0 : 1;
        }
        if (endsWith(firstArg, ".bphcl.yaml")) {
            std::string outPath = firstArg.substr(0, firstArg.size() - 5); // remove ".yaml"
            return convertYamlToBphcl(firstArg, outPath) ? 0 : 1;
        }
        if (endsWith(firstArg, ".bphnm")) {
            std::string outDir = std::filesystem::path(firstArg).parent_path().string();
            if (outDir.empty()) outDir = ".";
            return convertBphnmToYaml(firstArg, outDir) ? 0 : 1;
        }
        if (endsWith(firstArg, ".bphnm.yaml")) {
            std::string outPath = firstArg.substr(0, firstArg.size() - 5); // remove ".yaml"
            return convertYamlToBphnm(firstArg, outPath) ? 0 : 1;
        }
        if (endsWith(firstArg, ".bphnm.obj")) {
            // OBJ import: look for companion YAML, load it, then patch vertices from OBJ
            std::string yamlPath = firstArg.substr(0, firstArg.size() - 4) + ".yaml"; // .obj -> .yaml
            std::string outPath = firstArg.substr(0, firstArg.size() - 4); // remove ".obj"
            if (!std::filesystem::exists(yamlPath)) {
                std::cerr << "Companion YAML file required: " << yamlPath << "\n";
                return 1;
            }
            // Load YAML first to restore all binary data
            std::ifstream yamlFile(yamlPath);
            if (!yamlFile) { std::cerr << "Failed to open yaml file: " << yamlPath << "\n"; return 1; }
            std::string yamlStr((std::istreambuf_iterator<char>(yamlFile)), std::istreambuf_iterator<char>());
            std::vector<u8> yamlData(yamlStr.begin(), yamlStr.end());

            FiveX::NavMeshInfo info;
            info.loadFromYaml(yamlData);

            // Load OBJ and patch vertex positions
            std::ifstream objFile(firstArg, std::ios::binary | std::ios::ate);
            if (!objFile) { std::cerr << "Failed to open OBJ file: " << firstArg << "\n"; return 1; }
            std::streamsize objSize = objFile.tellg();
            objFile.seekg(0, std::ios::beg);
            std::vector<u8> objData((size_t)objSize);
            objFile.read(reinterpret_cast<char*>(objData.data()), objSize);

            if (!info.loadFromObj(objData)) {
                std::cerr << "Failed to apply OBJ vertices to navmesh\n";
                return 1;
            }

            // Serialize to bphnm
            std::vector<u8> outData;
            info.serializeToBphnm(outData);

            std::ofstream outFile(outPath, std::ios::binary);
            if (!outFile) { std::cerr << "Failed to create output: " << outPath << "\n"; return 1; }
            outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());
            std::cout << "Saved bphnm to " << std::filesystem::absolute(outPath) << "\n";
            return 0;
        }
        if (endsWith(firstArg, ".nvt")) {
            std::string outDir = std::filesystem::path(firstArg).parent_path().string();
            if (outDir.empty()) outDir = ".";
            return convertNvtToYaml(firstArg, outDir) ? 0 : 1;
        }
        if (endsWith(firstArg, ".nvt.yaml")) {
            std::string outPath = firstArg.substr(0, firstArg.size() - 5); // remove ".yaml"
            return convertYamlToNvt(firstArg, outPath) ? 0 : 1;
        }
        if (endsWith(firstArg, ".nvt.obj")) {
            // OBJ import: look for companion YAML, load it, then patch waypoints from OBJ
            std::string yamlPath = firstArg.substr(0, firstArg.size() - 4) + ".yaml"; // .obj -> .yaml
            std::string outPath = firstArg.substr(0, firstArg.size() - 4); // remove ".obj"
            if (!std::filesystem::exists(yamlPath)) {
                std::cerr << "Companion YAML file required: " << yamlPath << "\n";
                return 1;
            }
            std::ifstream yamlFile(yamlPath);
            if (!yamlFile) { std::cerr << "Failed to open yaml file: " << yamlPath << "\n"; return 1; }
            std::string yamlStr((std::istreambuf_iterator<char>(yamlFile)), std::istreambuf_iterator<char>());
            std::vector<u8> yamlData(yamlStr.begin(), yamlStr.end());

            FiveX::NvtInfo info;
            info.loadFromYaml(yamlData);

            std::ifstream objFile(firstArg, std::ios::binary | std::ios::ate);
            if (!objFile) { std::cerr << "Failed to open OBJ file: " << firstArg << "\n"; return 1; }
            std::streamsize objSize = objFile.tellg();
            objFile.seekg(0, std::ios::beg);
            std::vector<u8> objData((size_t)objSize);
            objFile.read(reinterpret_cast<char*>(objData.data()), objSize);

            if (!info.loadFromObj(objData)) {
                std::cerr << "Failed to apply OBJ waypoints to navtable\n";
                return 1;
            }

            std::vector<u8> outData;
            info.serializeToNvt(outData);

            std::ofstream outFile(outPath, std::ios::binary);
            if (!outFile) { std::cerr << "Failed to create output: " << outPath << "\n"; return 1; }
            outFile.write(reinterpret_cast<const char*>(outData.data()), outData.size());
            std::cout << "Saved nvt to " << std::filesystem::absolute(outPath) << "\n";
            return 0;
        }
    }

    if (!loadPhiveConfig()) {
        std::cerr << "Failed to load phive config!\n";
        return 1;
    }

    std::string phivePath, objPath, matPath, outPath;
    bool isPhiveToObj = false, isObjToPhive = false;

    if (argv[1][0] != '-') {
        std::vector<std::string> files;
        for (int i = 1; i < argc; i++) files.push_back(argv[i]);

        if (files.size() == 1) {
            if (endsWith(files[0], ".bphsh")) { isPhiveToObj = true; phivePath = files[0]; }
            else if (endsWith(files[0], ".obj")) { isObjToPhive = true; objPath = files[0]; }
            else { std::cerr << "Unknown file type: " << files[0] << "\n"; return 1; }
        } else if (files.size() == 2) {
            if ((endsWith(files[0], ".obj") && endsWith(files[1], ".json")) ||
                (endsWith(files[0], ".json") && endsWith(files[1], ".obj"))) {
                isObjToPhive = true;
                objPath = endsWith(files[0], ".obj") ? files[0] : files[1];
                matPath = endsWith(files[0], ".json") ? files[0] : files[1];
            } else { std::cerr << "Unknown file combination\n"; return 1; }
        }
    } else {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "-p" && i + 1 < argc) { phivePath = argv[++i]; isPhiveToObj = true; }
            else if (arg == "-obj" && i + 1 < argc) { objPath = argv[++i]; isObjToPhive = true; }
            else if (arg == "-mat" && i + 1 < argc) { matPath = argv[++i]; }
            else if (arg == "-o" && i + 1 < argc) { outPath = argv[++i]; }
        }
    }

    if (isPhiveToObj) {
        if (outPath.empty()) outPath = std::filesystem::path(phivePath).parent_path().string();
        if (!convertPhiveToObj(phivePath, outPath)) return 1;
    }

    if (isObjToPhive) {
        if (outPath.empty()) outPath = std::filesystem::path(objPath).replace_extension(".bphsh").string();
        if (!convertObjToPhive(objPath, matPath, outPath)) return 1;
    }

    return 0;
}
