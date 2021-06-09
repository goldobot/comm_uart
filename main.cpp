//#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <signal.h>

#include <unistd.h>

#include <string>
#include <iostream>

#include "zmq_sockets.h"

#include "optionparser.h"

#include "goldobot/comm_serializer.hpp"
#include "goldobot/comm_deserializer.hpp"


int terminal_parse_speed(std::string speed_str);

void close_zmq();
bool init_zmq(uint16_t pub_port, uint16_t sub_port);

int set_interface_attribs(int fd, int speed);
void set_mincount(int fd, int mcount);
int terminal_bytes_in_buffer(int fd);

volatile int keep_running = 1;

static void sigint_handler(int sig)
{
    keep_running = 0;
}

// pub port default 3001
// sub port default 3002

enum  optionIndex { UNKNOWN, HELP, PUB_PORT, SUB_PORT , BAUDRATE, DEVICE};
const option::Descriptor usage[] =
{
 {UNKNOWN, 0, "", "",option::Arg::None, "Usage: comm_uart [Options] device_path\n\n"
                                        "Options:" },
 {HELP, 0,"", "help",option::Arg::None, "  --help  \tPrint usage and exit." },
 {PUB_PORT, 0,"p","pub",option::Arg::Optional, "  --pub, -p  \tPUB port number." },
 {SUB_PORT, 0,"s","sub",option::Arg::Optional, "  --sub, -s  \tSUB port number." },
 {BAUDRATE, 0,"b","baudrate",option::Arg::Optional, "  --baudrate, -b  \tuart baudrate." },
 {DEVICE, 0,"d","device",option::Arg::Optional, "  --device, -d  \tdevice." },
 {UNKNOWN, 0, "", "",option::Arg::Optional, "\nExamples:\n"
                               "  example --unknown -- --this_is_no_option\n"
                               "  example -unk --plus -ppp file1 file2\n" },
 {0,0,0,0,0,0}
};

int main(int argc, char *argv[])
{
    argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present
    
    option::Stats  stats(usage, argc, argv);
    option::Option options[stats.options_max], buffer[stats.buffer_max];
    option::Parser parse(usage, argc, argv, options, buffer);
    
    if (parse.error())
        return 1;
    
    if (options[HELP] || argc == 0) {
        option::printUsage(std::cout, usage);
        return 0;
    }
    
    uint16_t pub_port = 3001;    
    if (options[PUB_PORT])
    {        
        pub_port = std::atoi(options[PUB_PORT].arg);
    };
    
    uint16_t sub_port = 3002;    
    if (options[SUB_PORT])
    {        
        sub_port = std::atoi(options[SUB_PORT].arg);
    };
    
    std::string speed = "230400";
    if (options[BAUDRATE])
    {        
        speed = std::string(options[BAUDRATE].arg);
    };
    
    std::string device_path = "";
    if (options[DEVICE])
    {        
        device_path = std::string(options[DEVICE].arg);
    };
    
    std::cout << "PUB port: " << pub_port << " SUB port: " << sub_port << "\n";
    std::cout << "device: " << device_path << " baudrate: " << speed << "\n";
    
    signal(SIGINT, sigint_handler);
    
    int fd;
   
    fd = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC );

    if (fd<0) {
        printf("Cannot open uart device (%s)\n", device_path.c_str());
        return -1;
    }
    
    int baudrate_i = terminal_parse_speed(speed);
    set_interface_attribs(fd, baudrate_i);
    
    init_zmq(pub_port, sub_port);
    
    zmq_pollitem_t poll_items[3];

    poll_items[0].socket = 0;
    poll_items[0].fd = fd;
    poll_items[0].events = ZMQ_POLLIN;

    poll_items[1].socket = sub_socket;
    poll_items[1].fd = 0;
    poll_items[1].events = ZMQ_POLLIN;
    
    poll_items[2].socket = 0;
    poll_items[2].fd = fd;
    poll_items[2].events = ZMQ_POLLOUT;

    unsigned char recv_buffer[4096];
    unsigned char serialize_buffer[4096];
    unsigned char tmp_buffer[1024];

    goldobot::CommDeserializer deserializer(recv_buffer, sizeof(recv_buffer));
    goldobot::CommSerializer serializer(serialize_buffer, sizeof(serialize_buffer));

    while(keep_running)
    {
        zmq_poll(poll_items, 2, 10);

        // Read data from uart
        if(poll_items[0].revents && ZMQ_POLLIN)
        {
            int rdlen;

            rdlen = read(fd, tmp_buffer, sizeof(tmp_buffer));
            deserializer.push_data(tmp_buffer, rdlen);

            while(deserializer.message_ready())
            {
                uint16_t recv_message_type = deserializer.message_type();
                size_t recv_message_size = deserializer.message_size();
                deserializer.pop_message(tmp_buffer, sizeof(tmp_buffer));

                zmq_send(pub_socket, (const char*)(&recv_message_type), 2, ZMQ_SNDMORE);
                zmq_send(pub_socket, (const char*)(tmp_buffer), recv_message_size, 0);
            }
        }
        
        // Read message from zmq
        if(poll_items[1].revents && ZMQ_POLLIN)
        {
            unsigned char buff[1024];
            size_t bytes_read = 0;
            int64_t more=1;
            size_t more_size = sizeof(more);
            while(more)
            {
                bytes_read += zmq_recv(sub_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);
                zmq_getsockopt(sub_socket, ZMQ_RCVMORE, &more, &more_size);
            }
            buff[bytes_read] = 0;
            uint16_t message_type = *(uint16_t*)(buff);
            serializer.push_message(message_type, buff+2, bytes_read - 2);
        }
        
        // Send data to uart
        {
            int bytes_in_buffer = terminal_bytes_in_buffer(fd);
            
            if(serializer.size() > 0 && bytes_in_buffer < 1024)
            {   
                size_t dlen = serializer.pop_data(tmp_buffer, sizeof(tmp_buffer));
                write(fd, tmp_buffer, dlen);
            }
        }
    }
    
    printf("close zmq sockets\n");
    close_zmq(),
    close(fd);    
}

