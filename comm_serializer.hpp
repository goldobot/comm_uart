#pragma once
#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class CommSerializer
	{
	public:
		CommSerializer(unsigned char* buffer, size_t size);

		size_t size();

		bool push_message(uint16_t message_type, unsigned char* buffer, size_t size);
		size_t pop_data(unsigned char* buffer, size_t size);

	private:
		void push_data(unsigned char* buffer, size_t size);

		unsigned char* m_buffer;
		size_t m_buffer_size;
		size_t m_begin_index;
		size_t m_end_index;
	};

	class CommDeserializer
	{
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
	};
}
