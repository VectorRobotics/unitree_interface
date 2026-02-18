#ifndef VECTOR_CRC_HPP
#define VECTOR_CRC_HPP

#include <cstdint>

namespace unitree_interface {

    inline std::uint32_t crc_32_core(const std::uint32_t* ptr, std::uint32_t len) noexcept {
        std::uint32_t xbit = 0;
        std::uint32_t data = 0;
        std::uint32_t crc32 = 0xFFFFFFFF; // NOLINT

        const std::uint32_t dw_polynomial = 0x04c11db7;
        for (std::uint32_t i = 0; i < len; i++) {
            xbit = 1 << 31; // NOLINT
            data = ptr[i];

            for (uint32_t bits = 0; bits < 32; bits++) { // NOLINT
                if (crc32 & 0x80000000) { // NOLINT
                    crc32 <<= 1;
                    crc32 ^= dw_polynomial;
                } else {
                    crc32 <<= 1;
                }

                if ((data & xbit) != 0) {
                    crc32 ^= dw_polynomial;
                }

                xbit >>= 1;
            }
        }
        return crc32;
    }

} // namespace unitree_interface

#endif // VECTOR_CRC_HPP
