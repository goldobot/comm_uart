#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>

#include "comm_serializer.hpp"

void* zmq_context = nullptr;
void* pub_socket = nullptr;
void* sub_socket = nullptr;

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


void init_zmq()
{
    int rc;
    zmq_context = zmq_init(1);

    pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
    rc = zmq_bind(pub_socket, "tcp://*:3001");
    if(rc != 0) printf("failed to bind pub socket");

    sub_socket = zmq_socket(zmq_context, ZMQ_SUB);  
    rc = zmq_bind(sub_socket, "tcp://*:3002");
    zmq_setsockopt(sub_socket,ZMQ_SUBSCRIBE, "", 0); 
    if(rc != 0) printf("failed to bind sub socket");
};

int main(int argc, char *argv[])
{
    char *portname = "/dev/ttyACM0";
    int fd;
    
    init_zmq();

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd<0) {
        printf("Cannot open uart device (%s)\n", portname);
        return -1;
    }
    set_interface_attribs(fd, B230400);    

    zmq_pollitem_t poll_items[2];

    poll_items[0].socket = 0;
    poll_items[0].fd = fd;
    poll_items[0].events = ZMQ_POLLIN;

    poll_items[1].socket = sub_socket;
    poll_items[1].fd = 0;
    poll_items[1].events = ZMQ_POLLIN;

    unsigned char recv_buffer[4096];
    unsigned char serialize_buffer[4096];
    unsigned char tmp_buffer[1024];
    goldobot::CommDeserializer deserializer(recv_buffer, sizeof(recv_buffer));
    goldobot::CommSerializer serializer(serialize_buffer, sizeof(serialize_buffer));
    while(1)
    {
        zmq_poll (poll_items, 2, -1);

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

                if(recv_message_type == 0)
                {
                    // Echo synchronization messages
                    serializer.push_message(0, (unsigned char*)"goldobot", 8);
                }
            }
        }
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
        size_t dlen = serializer.pop_data(tmp_buffer, sizeof(tmp_buffer));
        write(fd, tmp_buffer, dlen);
    }
    zmq_term(zmq_context);
}
