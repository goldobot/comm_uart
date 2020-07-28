all:comm_uart

comm_uart:
	g++ main.cpp comm_serializer.cpp comm_deserializer.cpp crc.cpp -std=c++11 -lzmq -o comm_uart

clean:
	rm -f *.o comm_uart
