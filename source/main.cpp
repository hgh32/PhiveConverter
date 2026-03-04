#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include "fivex/CollisionInfo.hpp"

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
    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "help") {
        std::cout << "Usage:\n"
                  << "  PhiveConverter.exe FILE.bphsh           - convert bphsh to obj+json\n"
                  << "  PhiveConverter.exe FILE.obj FILE.json   - convert obj+json to bphsh\n"
                  << "  PhiveConverter.exe FILE.obj             - convert obj to bphsh (no mat info)\n"
                  << "  PhiveConverter.exe -p PHIVE -o OUTDIR   - phive to obj\n"
                  << "  PhiveConverter.exe -obj OBJ -mat MAT -o OUT - obj to phive\n";
        return 0;
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
