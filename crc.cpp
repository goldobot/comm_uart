
#include "goldobot/utils/crc.hpp"

uint16_t update_crc16(const unsigned char* data_p, size_t length, uint16_t crc)
{
    unsigned char x;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x << 5)) ^ ((unsigned short)x);
    }
    return crc;
}