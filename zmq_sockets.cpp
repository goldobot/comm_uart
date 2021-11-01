#include "zmq_sockets.h"
#include <sstream>

void* zmq_context = nullptr;
void* pub_socket = nullptr;
void* sub_socket = nullptr;

bool init_zmq(uint16_t pub_port, uint16_t sub_port, uint8_t comm_id)
{
    int rc;
    zmq_context = zmq_init(1);      
    
    if(zmq_context == 0)
    {
        printf("failed to create zmq context\n");
        return false;
    }
    
    std::ostringstream stream;
    stream << "tcp://*:" << pub_port;
    std::string str = stream.str();
    
    pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
    rc = zmq_bind(pub_socket, str.c_str());
    
    if(rc != 0) 
    {
        printf("failed to bind pub socket\n");
        zmq_close(pub_socket);
        zmq_term(zmq_context);
        return false;
    }
    stream = std::ostringstream();
    stream << "tcp://*:" << sub_port;
    str = stream.str();
    
    sub_socket = zmq_socket(zmq_context, ZMQ_SUB);  
    rc = zmq_bind(sub_socket, str.c_str());
    zmq_setsockopt(sub_socket,ZMQ_SUBSCRIBE, &comm_id, 1); 
    
    if(rc != 0) 
    {
        printf("failed to bind sub socket\n");
        zmq_close(pub_socket);
        zmq_close(sub_socket);
        zmq_term(zmq_context);
        return false;
    }
    return true;
};

void close_zmq()
{
    zmq_close(pub_socket);
    zmq_close(sub_socket);
    zmq_term(zmq_context);
}