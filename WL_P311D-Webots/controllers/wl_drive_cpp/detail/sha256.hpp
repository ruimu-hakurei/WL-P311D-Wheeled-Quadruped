#pragma once

// 文件指纹的内部实现。控制算法只使用 sha256File()，不直接使用此类。
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace wl::detail {

class Sha256 {
public:
    void update(const unsigned char* bytes, std::size_t length);
    std::string finish();

private:
    using Word = std::uint32_t;
    using MessageSchedule = std::array<Word, 64>;

    MessageSchedule buildMessageSchedule() const;
    void compressBlock();
    void padFinalBlock();
    std::string formatDigest() const;

    std::array<Word, 8> state_{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                               0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::array<unsigned char, 64> block_{};
    std::size_t bytes_in_block_ = 0;
    std::uint64_t total_bytes_ = 0;
};

}  // namespace wl::detail
