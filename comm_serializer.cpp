#include "comm_serializer.hpp"
#include <iostream>

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

size_t read_varint(uint16_t* val, unsigned char* buffer)
{
    if (!(buffer[0] & 0x80))
    {
        *val = buffer[0];
        return 1;
    }
    else
    {
        uint16_t decoded = (buffer[0] & 0x7F);
        decoded = (decoded << 8) | buffer[1];
        *val = decoded;
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

bool CommSerializer::push_message(uint16_t message_type, unsigned char* buffer, size_t size)
{
    unsigned char header[4];
    unsigned char* ptr = header;
    ptr += write_varint(message_type, ptr);
    ptr += write_varint(size, ptr);

    // Compute crc
    uint16_t crc = update_crc16(header, ptr-header);
    crc = update_crc16(buffer, size, crc);

    // Copy data into ring buffer
    push_data(header, ptr-header);
    push_data(buffer, size);
    push_data((unsigned char*)(&crc), sizeof(crc));

    return true;
}

void CommSerializer::push_data(unsigned char* buffer, size_t size)
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

size_t CommSerializer::size()
{
    if(m_end_index >= m_begin_index)
    {
        return m_end_index - m_begin_index;
    } else
    {
        return m_end_index - m_begin_index + m_buffer_size;
    }
}



CommDeserializer::CommDeserializer(unsigned char* buffer, size_t buffer_size):
	m_buffer(buffer),
	m_buffer_size(buffer_size),
	m_begin_index(0),
	m_end_index(0),
	m_state(SearchMagic)
{

}

size_t CommDeserializer::push_data(unsigned char* buffer, size_t size)
{
	for (unsigned i = 0; i< size; i++)
	{
		m_buffer[m_end_index] = buffer[i];
		m_end_index++;
		if (m_end_index == m_buffer_size)
		{
			m_end_index = 0;
		}
	}
	do_parse();
	return size;
}

size_t CommDeserializer::size() const
{
	return m_end_index >= m_begin_index ? m_end_index - m_begin_index : m_end_index - m_begin_index + m_buffer_size;
}

bool CommDeserializer::message_ready() const
{
	return m_state == MessageReady;
}

uint16_t CommDeserializer::message_type() const
{
	return m_message_type;
}
size_t CommDeserializer::message_size() const
{
	return m_message_size;
}

void CommDeserializer::read_data(size_t start_index, unsigned char* buffer, size_t size)
{
	size_t i = 0;
	size_t idx = start_index;
	if(idx > m_buffer_size)
	{
		idx -= m_buffer_size;
	}
	while (i < size)
	{
		buffer[i] = m_buffer[idx];
		i++;
		idx++;
		if (idx == m_buffer_size)
		{
			idx = 0;
		}
	}
}

void CommDeserializer::pop_data(size_t size)
{
	m_begin_index += size;
	if (m_begin_index >= m_buffer_size)
	{
		m_begin_index -= m_buffer_size;
	}
}

void CommDeserializer::pop_message(unsigned char* buffer, size_t size)
{
	read_data(m_begin_index, buffer, m_message_size);
	pop_data(m_message_size + 2);
	m_state = ReadHeader;

	do_parse();
}

void CommDeserializer::do_parse()
{
	unsigned char magic[] = "goldobot\132\34";
	magic[8] = 132;
	magic[9] = 34;
	while (m_begin_index != m_end_index)
	{
		switch (m_state)
		{
		case SearchMagic:
		{
			if (size() < sizeof(magic) - 1)
			{
				return;
			}

			size_t idx = m_begin_index;
			unsigned j = 0;
			while(j < sizeof(magic)-1 && idx != m_end_index)
			{
				if (m_buffer[idx] == magic[j])
				{
					j++;
					idx++;
					if(idx == m_buffer_size)
					{
						idx = 0;
					}
				}
				else
				{
					break;
				}
			}
			std::cout << "search magic: " <<j <<   std::endl;
			if(j == sizeof(magic)-1)
			{
				std::cout << "synchronized" << std::endl;
				m_state = ReadHeader;
				m_begin_index = idx;
			}
			else
			{
				m_begin_index++;
				if (m_begin_index == m_buffer_size)
				{
					m_begin_index = 0;
				}
			}

		}
			break;

		case ReadHeader:
		{
			// Header is at most 4 bytes
			if (size() < 4)
			{
				return;
			}
			unsigned char buff[4];
			read_data(m_begin_index, buff, 4);

			// Decode header data
			unsigned char* ptr = buff;
			ptr += read_varint(&m_message_type, ptr);
			ptr += read_varint(&m_message_size, ptr);

			// Compute crc of header
			m_crc = update_crc16(buff, ptr - buff);

			// Advance fifo at beginning of message
			pop_data(ptr - buff);
			// Goto next stage
			m_state = ReadBody;
		}
		break;
		case ReadBody:
		{
			// Check for reception of whole message
			if (size() >= m_message_size + 2)
			{
				// Read crc
				unsigned char buff[2];
				read_data(m_begin_index + m_message_size, buff, 2);

				// Update crc with that of message body
				if (m_begin_index + m_message_size <= m_buffer_size)
				{
					m_crc = update_crc16(m_buffer + m_begin_index, m_message_size, m_crc);
				}
				else
				{
					// Update in two steps if the message straddle the circular buffer end
					size_t first_chunk_size = m_buffer_size - m_begin_index;
					m_crc = update_crc16(m_buffer + m_begin_index, first_chunk_size, m_crc);
					m_crc = update_crc16(m_buffer, m_message_size - first_chunk_size, m_crc);
				}
				// Check crc
				if (m_crc == *(uint16_t*)(buff))
				{
					m_state = MessageReady;
				}
				else
				{
					m_state = SearchMagic;
				}
			}
			else
			{
				return;
			}
		}
		break;
		case MessageReady:
			return;
		default:
			return;
		}
	}
}
