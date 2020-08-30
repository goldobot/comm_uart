#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>

#include "goldobot/comm_serializer.hpp"
#include "goldobot/comm_deserializer.hpp"

void* zmq_context = nullptr;
void* pub_socket = nullptr;
void* sub_socket = nullptr;

volatile int keep_running = 1;

static void sigint_handler(int sig)
{
    keep_running = 0;
}

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
    if(rc != 0) 
    {
        printf("failed to bind pub socket\n");
        _exit(0);
    }
    sub_socket = zmq_socket(zmq_context, ZMQ_SUB);  
    rc = zmq_bind(sub_socket, "tcp://*:3002");
    zmq_setsockopt(sub_socket,ZMQ_SUBSCRIBE, "", 0); 
    if(rc != 0) printf("failed to bind sub socket\n");
};

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        printf("Usage: comm_uart device_path [baudrate, default=230400]\n");
        return 0;
    };
    signal(SIGINT, sigint_handler);

    

    int speed = B230400;
    if(argc == 3)
    {
        speed = 0;
        if(strcmp(argv[2], "4800") == 0)
        {
            speed = B4800;
        } 
        if(strcmp(argv[2], "9600") == 0)
        {
            speed = B9600;
        } 
        if(strcmp(argv[2], "19200") == 0)
        {
            speed = B19200;
        }
        if(strcmp(argv[2], "38400") == 0)
        {
            speed = B38400;
        }
        if(strcmp(argv[2], "57600") == 0)
        {
            speed = B57600;
        }
        if(strcmp(argv[2], "115200") == 0)
        {
            speed = B115200;
        }
        if(strcmp(argv[2], "230400") == 0)
        {
            speed = B230400;
        }
        if(strcmp(argv[2], "460800 ") == 0)
        {
            speed = B460800;
        }
        if(strcmp(argv[2], "500000") == 0)
        {
            speed = B500000;
        }
        if(strcmp(argv[2], "576000") == 0)
        {
            speed = B576000;
        }
        if(strcmp(argv[2], "921600") == 0)
        {
            speed = B921600;
        }
        if(strcmp(argv[2], "1000000") == 0)
        {
            speed = B1000000;
        }
        if(speed ==0 )
        {
            printf("Invalid baudrate\n");
            return -1;
        }
    }
    
    int fd;
   
    fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC );

    if (fd<0) {
        printf("Cannot open uart device (%s)\n", argv[1]);
        return -1;
    }
    
    set_interface_attribs(fd, speed);
    
     init_zmq();
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
            int bytes_in_buffer = 0;
            ioctl(fd, TIOCOUTQ, &bytes_in_buffer);
            
            if(serializer.size() > 0 && bytes_in_buffer < 1024)
            {   
                size_t dlen = serializer.pop_data(tmp_buffer, sizeof(tmp_buffer));
                write(fd, tmp_buffer, dlen);
            }
        }
    }
    printf("close zmq sockets\n");
    zmq_close(pub_socket);
    zmq_close(sub_socket);
    close(fd);
    zmq_term(zmq_context);
}

