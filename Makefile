all:comm_uart

comm_uart:
	g++ test.cpp comm_serializer.cpp -std=c++11 -lzmq -o comm_uart

clean:
	rm -f *.o comm_uart
