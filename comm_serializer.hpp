#pragma once
#include <cstdint>
#include <cstddef>

namespace goldobot
{
	class CommSerializer
	{
	public:
		CommSerializer(unsigned char* buffer, size_t size);

		size_t size() const;
		size_t availableSize() const;

		bool push_message(uint16_t message_type, const unsigned char* buffer, size_t size);
		size_t pop_data(unsigned char* buffer, size_t size);

	private:
		void push_data(const unsigned char* buffer, size_t size);

		unsigned char* m_buffer;
		size_t m_buffer_size;
		size_t m_begin_index;
		size_t m_end_index;
		uint8_t m_sequence_number{0};
	};
}
