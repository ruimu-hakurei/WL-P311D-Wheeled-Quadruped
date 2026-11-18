#include "sha256.hpp"

#include <iomanip>
#include <sstream>

namespace wl::detail {
namespace {

using Word = std::uint32_t;

// 标准 SHA-256 常量表。它不是控制参数，无需调整。
constexpr std::array<Word, 64> ROUND_CONSTANTS{
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

Word rotateRight(Word value, unsigned bit_count) {
    return (value >> bit_count) | (value << (32 - bit_count));
}

Word choose(Word first, Word second, Word third) {
    return (first & second) ^ ((~first) & third);
}

Word majority(Word first, Word second, Word third) {
    return (first & second) ^ (first & third) ^ (second & third);
}

Word largeSigma0(Word value) {
    return rotateRight(value, 2) ^ rotateRight(value, 13) ^ rotateRight(value, 22);
}

Word largeSigma1(Word value) {
    return rotateRight(value, 6) ^ rotateRight(value, 11) ^ rotateRight(value, 25);
}

Word smallSigma0(Word value) {
    return rotateRight(value, 7) ^ rotateRight(value, 18) ^ (value >> 3);
}

Word smallSigma1(Word value) {
    return rotateRight(value, 17) ^ rotateRight(value, 19) ^ (value >> 10);
}

}  // namespace

void Sha256::update(const unsigned char* bytes, std::size_t length) {
    total_bytes_ += length;
    for (std::size_t byte_index = 0; byte_index < length; ++byte_index) {
        block_[bytes_in_block_] = bytes[byte_index];
        ++bytes_in_block_;
        if (bytes_in_block_ == block_.size()) {
            compressBlock();
            bytes_in_block_ = 0;
        }
    }
}

Sha256::MessageSchedule Sha256::buildMessageSchedule() const {
    MessageSchedule words{};

    // 每四个字节按大端顺序组合成一个 32 位字。
    for (std::size_t word_index = 0; word_index < 16; ++word_index) {
        const std::size_t offset = 4 * word_index;
        words[word_index] = (Word(block_[offset]) << 24) | (Word(block_[offset + 1]) << 16)
                            | (Word(block_[offset + 2]) << 8) | block_[offset + 3];
    }

    // 根据 SHA-256 规则，把 16 个输入字扩展为 64 个轮次用字。
    for (std::size_t word_index = 16; word_index < words.size(); ++word_index) {
        words[word_index] = words[word_index - 16] + smallSigma0(words[word_index - 15])
                            + words[word_index - 7] + smallSigma1(words[word_index - 2]);
    }
    return words;
}

void Sha256::compressBlock() {
    const MessageSchedule words = buildMessageSchedule();

    // a 到 h 是 SHA-256 标准中的八个工作字。
    Word a = state_[0];
    Word b = state_[1];
    Word c = state_[2];
    Word d = state_[3];
    Word e = state_[4];
    Word f = state_[5];
    Word g = state_[6];
    Word h = state_[7];

    for (std::size_t round_index = 0; round_index < words.size(); ++round_index) {
        const Word round_sum1 = h + largeSigma1(e) + choose(e, f, g) + ROUND_CONSTANTS[round_index]
                                + words[round_index];
        const Word round_sum2 = largeSigma0(a) + majority(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + round_sum1;
        d = c;
        c = b;
        b = a;
        a = round_sum1 + round_sum2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
}

void Sha256::padFinalBlock() {
    const std::uint64_t original_bit_count = total_bytes_ * 8;
    block_[bytes_in_block_++] = 0x80;

    // 最后八字节保存原始长度；放不下时先处理当前块。
    if (bytes_in_block_ > 56) {
        while (bytes_in_block_ < block_.size()) {
            block_[bytes_in_block_++] = 0;
        }
        compressBlock();
        bytes_in_block_ = 0;
    }
    while (bytes_in_block_ < 56) {
        block_[bytes_in_block_++] = 0;
    }
    for (unsigned byte_index = 0; byte_index < 8; ++byte_index) {
        const unsigned shift = 56 - 8 * byte_index;
        block_[56 + byte_index] = static_cast<unsigned char>(original_bit_count >> shift);
    }
}

std::string Sha256::formatDigest() const {
    std::ostringstream digest;
    digest << std::hex << std::setfill('0');
    for (Word state_word : state_) {
        digest << std::setw(8) << state_word;
    }
    return digest.str();
}

std::string Sha256::finish() {
    padFinalBlock();
    compressBlock();
    return formatDigest();
}

}  // namespace wl::detail
