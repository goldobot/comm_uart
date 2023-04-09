all:comm_uart

comm_uart: main.cpp comm_serializer.cpp comm_deserializer.cpp crc.cpp zmq_sockets.cpp terminal.cpp
	g++ main.cpp comm_serializer.cpp comm_deserializer.cpp crc.cpp zmq_sockets.cpp terminal.cpp -std=c++11 -lzmq -o comm_uart

comm_uart_ftdi: main.cpp comm_serializer.cpp comm_deserializer.cpp crc.cpp zmq_sockets.cpp terminal.cpp
	g++ -DDEBUG_FULL main.cpp comm_serializer.cpp comm_deserializer.cpp crc.cpp zmq_sockets.cpp terminal.cpp -std=c++11 -lzmq -o comm_uart_ftdi

clean:
	rm -f *.o comm_uart comm_uart_ftdi
