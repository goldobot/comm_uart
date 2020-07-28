#include "goldobot/comm_deserializer.hpp"
#include "goldobot/utils/crc.hpp"

#include <algorithm>

using namespace goldobot;

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

size_t read_varint(size_t* val, unsigned char* buffer)
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

CommDeserializer::CommDeserializer(unsigned char* buffer, size_t buffer_size):
	m_buffer(buffer),
	m_buffer_size(buffer_size),
	m_begin_index(0),
	m_end_index(0),
	m_state(SearchMagic)
{

}

CommDeserializer::Statistics CommDeserializer::statistics()
{
	auto statistics = m_statistics;
	m_statistics = Statistics();
	return statistics;
}

size_t CommDeserializer::push_data(unsigned char* buffer, size_t buffer_size)
{
	size_t space_available = m_buffer_size - size() - 1;
	size_t bytes_received = std::min(space_available, buffer_size);

	for (unsigned i = 0; i< bytes_received; i++)
	{
		m_buffer[m_end_index] = buffer[i];
		m_end_index++;
		if (m_end_index == m_buffer_size)
		{
			m_end_index = 0;
		}
	}

	do_parse();
	m_statistics.bytes_received += bytes_received;
	m_statistics.buffer_high_watermark = std::max<uint32_t>(m_statistics.buffer_high_watermark, size());
	return bytes_received;
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
	if(idx >= m_buffer_size)
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
	while (m_begin_index >= m_buffer_size)
	{
		m_begin_index -= m_buffer_size;
	}
}

void CommDeserializer::pop_message(unsigned char* buffer, size_t size)
{
	if(buffer)
	{
		read_data(m_begin_index, buffer, m_message_size);
	}
	pop_data(m_message_size + 2);
	m_state = SearchMagic;

	do_parse();
}

void CommDeserializer::do_parse()
{
	unsigned char magic[] = { 0x0a, 0x35 };

	while (m_begin_index != m_end_index)
	{
		switch (m_state)
		{
		case SearchMagic:
		{
			if (size() < sizeof(magic))
			{
				return;
			}
			size_t idx = m_begin_index;
			unsigned j = 0;
			while(j < sizeof(magic))
			{
				if (idx == m_end_index)
				{
					int a = size();
					int b = sizeof(magic) - 1;
					int c = 1;
				}
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
			if(j == sizeof(magic))
			{
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
			// Header is at most 6 bytes
			if (size() < 6)
			{
				return;
			}
			unsigned char buff[6];
			read_data(m_begin_index, buff, 6);

			// Decode header data
			unsigned char* ptr = buff + 2;
            m_sequence_number = buff[0] & 0x7f;
			// reset sequence flag flag
			if (buff[0] & 0x80)
			{
				m_expected_sequence_number = m_sequence_number;
			}
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
					m_statistics.messages_received++;

					if (m_sequence_number != m_expected_sequence_number)
					{
						m_statistics.sequence_errors++;
					}
					m_expected_sequence_number = ((m_sequence_number + 1) & 0x7f);
				}
				else
				{
					m_statistics.crc_errors++;
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
