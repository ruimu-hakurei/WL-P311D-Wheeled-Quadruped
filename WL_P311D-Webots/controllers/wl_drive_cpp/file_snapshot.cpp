// 【文件辅助实现】分块读取避免整文件常驻内存；读取失败抛异常，由调用层决定处理。
#include "file_snapshot.hpp"
#include "detail/sha256.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace wl {

std::string sha256File(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Cannot open file for SHA256");
    }

    // 这里仅负责分块读取文件；哈希运算细节在 detail/sha256.cpp。
    detail::Sha256 digest;
    std::array<unsigned char, 8192> buffer{};
    while (input) {
        // reinterpret_cast 仅把字节缓冲区视为流接口需要的 char*，并不改变字节内容。
        input.read(reinterpret_cast<char*>(buffer.data()),
                   static_cast<std::streamsize>(buffer.size()));
        const auto bytes_read = static_cast<std::size_t>(input.gcount());
        digest.update(buffer.data(), bytes_read);
    }
    if (!input.eof()) {
        throw std::runtime_error("Cannot read file for SHA256");
    }
    return digest.finish();
}

std::string jsonString(const std::string& text) {
    std::ostringstream escaped;
    escaped << '"';
    for (unsigned char character : text) {
        switch (character) {
            case '"':
                escaped << "\\\"";
                break;
            case '\\':
                escaped << "\\\\";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            case '\t':
                escaped << "\\t";
                break;
            default:
                if (character < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<unsigned>(character) << std::dec;
                } else {
                    escaped << static_cast<char>(character);
                }
                break;
        }
    }
    escaped << '"';
    return escaped.str();
}

}  // namespace wl
