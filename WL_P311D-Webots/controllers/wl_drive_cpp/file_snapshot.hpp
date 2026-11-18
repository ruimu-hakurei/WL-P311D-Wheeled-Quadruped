// 【文件辅助接口】SHA-256 文件指纹与 JSON 文本转义，和机器人控制数学无关。
#pragma once

#include <filesystem>
#include <string>

namespace wl {

// 启动时生成源文件指纹；底层哈希运算位于 detail/sha256.cpp。
std::string sha256File(const std::filesystem::path& path);
std::string jsonString(const std::string& text);

}  // namespace wl
