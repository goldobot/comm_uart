#pragma once
#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class CommDeserializer
	{
	public:
		struct Statistics
		{
			uint32_t sequence_errors{0};
		};

	public:
	    CommDeserializer(unsigned char* buffer, size_t buffer_size);

	    size_t push_data(unsigned char* buffer, size_t size);

	    bool message_ready() const;
	    uint16_t message_type() const;
	    size_t message_size() const;
	    void pop_message(unsigned char* buffer, size_t size);

	private:
		enum State
		{
			SearchMagic,
			ReadHeader,
			ReadBody,
			MessageReady
		};

		size_t size() const;
		void read_data(size_t start_index, unsigned char* buffer, size_t size);
		void pop_data(size_t size);
		void do_parse();

	    unsigned char* m_buffer;
	    size_t m_buffer_size;

		size_t m_begin_index;
		size_t m_end_index;

	    uint16_t m_message_size;
	    uint16_t m_message_type;
	    uint16_t m_state;
		uint16_t m_crc;
		uint8_t m_sequence_number{0};
		uint8_t m_last_sequence_number{0};

		Statistics m_statistics;
		int m_corrupted_messages{0};
		int m_lost_messages{0};
	};
}
