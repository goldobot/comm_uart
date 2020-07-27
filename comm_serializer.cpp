#include "comm_serializer.hpp"

using namespace goldobot;

uint16_t update_crc16(const unsigned char* data_p, size_t length, uint16_t crc = 0xFFFF)
{
    unsigned char x;

    while (length--) {
        x = crc >> 8 ^ *data_p++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x << 5)) ^ ((unsigned short)x);
    }
    return crc;
}

size_t write_varint(uint16_t val, unsigned char* buffer)
{
    if (val < 128)
    {
        buffer[0] = val & 0x7F;
        return 1;
    }
    else
    {
        buffer[0] = ((val >> 8) & 0x7F) | 0x80;
        buffer[1] = val & 0x00FF;
        return 2;
    }
}

CommSerializer::CommSerializer(unsigned char* buffer, size_t size):
    m_buffer(buffer),
    m_buffer_size(size),
    m_begin_index(0),
    m_end_index(0)
{

}

bool CommSerializer::push_message(uint16_t message_type, const unsigned char* buffer, size_t msg_size)
{
	// Reject message if buffer is full
	if(size() + msg_size + 12 >= m_buffer_size)
	{
		return false;
	}
    unsigned char header[6];
    unsigned char* ptr = header + 2;
    header[0] = 0x80 | (m_sequence_number & 0x7f);
    header[1] = 0x80;
    ptr += write_varint(message_type, ptr);
    ptr += write_varint(msg_size, ptr);

    // Compute crc
    uint16_t crc = update_crc16(header, ptr-header);
    crc = update_crc16(buffer, msg_size, crc);

    // Copy data into ring buffer
    push_data(header, ptr-header);
    push_data(buffer, msg_size);
    push_data((unsigned char*)(&crc), sizeof(crc));

    m_sequence_number = (m_sequence_number + 1) & 0x7f;
    return true;
}

void CommSerializer::push_data(const unsigned char* buffer, size_t size)
{
    for(unsigned i=0; i< size; i++)
    {
        m_buffer[m_end_index] = buffer[i];
        m_end_index++;
        if(m_end_index == m_buffer_size)
        {
            m_end_index = 0;
        }
    }
}

size_t CommSerializer::pop_data(unsigned char* buffer, size_t size)
{
	size_t i = 0;
	while (m_begin_index != m_end_index && i < size)
	{
		buffer[i] = m_buffer[m_begin_index];
		i++;
		m_begin_index++;
		if (m_begin_index == m_buffer_size)
		{
			m_begin_index = 0;
		}
	}
	return i;
}

size_t CommSerializer::size() const
{
    if(m_end_index >= m_begin_index)
    {
        return m_end_index - m_begin_index;
    } else
    {
        return m_end_index - m_begin_index + m_buffer_size;
    }
}

size_t CommSerializer::availableSize() const
{
	return m_buffer_size - (size() + 12);
}
