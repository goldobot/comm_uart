#include <zmq.h>
#include <cstdint>

extern void* zmq_context;
extern void* pub_socket;
extern void* sub_socket;

bool init_zmq(uint16_t pub_port, uint16_t sub_port, uint8_t comm_id);
