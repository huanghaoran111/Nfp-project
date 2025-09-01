#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

std::vector<std::string> RefreshJsonFileList(const std::string& folderPath) {
    std::vector<std::string> jsonFiles;
    
    try {
        // 遍历目录
        for (const auto& entry : fs::directory_iterator(folderPath)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                // 检查文件扩展名是否为.json
                if (filename.size() > 5 && 
                    filename.substr(filename.size() - 5) == ".json") {
                    jsonFiles.push_back(filename);
                }
            }
        }
        
        // 对文件名排序
        std::sort(jsonFiles.begin(), jsonFiles.end());
    } catch (const fs::filesystem_error& e) {
        // 处理文件系统错误，例如目录不存在
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    }
    
    return jsonFiles;
}