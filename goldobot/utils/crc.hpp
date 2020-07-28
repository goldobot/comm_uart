#include <cstdint>
#include <cstddef>

uint16_t update_crc16(const unsigned char* data_p, size_t length, uint16_t crc = 0xFFFF);
