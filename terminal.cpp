#include <errno.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>

#include <string>

int terminal_parse_speed(std::string speed_str)
{
    if(speed_str == "4800")
    {
        return B4800;
    } 
    if(speed_str == "9600")
    {
        return B9600;
    } 
    if(speed_str == "19200")
    {
        return B19200;
    }
    if(speed_str == "38400")
    {
        return B38400;
    }
    if(speed_str == "57600")
    {
        return B57600;
    }
    if(speed_str == "115200")
    {
        return B115200;
    }
    if(speed_str == "230400")
    {
        return B230400;
    }
    if(speed_str == "460800 ")
    {
        return B460800;
    }
    if(speed_str == "500000")
    {
        return B500000;
    }
    if(speed_str == "576000")
    {
        return B576000;
    }
    if(speed_str == "921600")
    {
        return B921600;
    }
    if(speed_str == "1000000")
    {
        return B1000000;
    }
    printf("Invalid baudrate\n");
    return -1; 
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

int terminal_bytes_in_buffer(int fd)
{    
    int bytes_in_buffer = 0;
    ioctl(fd, TIOCOUTQ, &bytes_in_buffer);
    return bytes_in_buffer;
}
