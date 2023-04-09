//#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include <unistd.h>

#include <string>
#include <iostream>

#include "zmq_sockets.h"

#include "optionparser.h"

#include "goldobot/comm_serializer.hpp"
#include "goldobot/comm_deserializer.hpp"

//#define DEBUG_FULL 1
//#define DEBUG_HEARTBEAT 1
//#define DEBUG_ADC 1

int terminal_parse_speed(std::string speed_str);

void close_zmq();
bool init_zmq(uint16_t pub_port, uint16_t sub_port, uint8_t comm_id);

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

enum  optionIndex { UNKNOWN, HELP, PUB_PORT, SUB_PORT , BAUDRATE, DEVICE, COMM_ID, DEBUG};
const option::Descriptor usage[] =
{
 {UNKNOWN, 0, "", "",option::Arg::None, "Usage: comm_uart [Options] device_path\n\n"
                                        "Options:" },
 {HELP, 0,"", "help",option::Arg::None, "  --help  \tPrint usage and exit." },
 {PUB_PORT, 0,"p","pub",option::Arg::Optional, "  --pub, -p  \tPUB port number." },
 {SUB_PORT, 0,"s","sub",option::Arg::Optional, "  --sub, -s  \tSUB port number." },
 {BAUDRATE, 0,"b","baudrate",option::Arg::Optional, "  --baudrate, -b  \tuart baudrate." },
 {DEVICE, 0,"d","device",option::Arg::Optional, "  --device, -d  \tdevice." },
 {COMM_ID, 0,"i","id",option::Arg::Optional, "  --id, -i  \tcomm ide." },
 {DEBUG, 0,"D","debug",option::Arg::None, "  --debug, -D  \tDEBUG." },
 {UNKNOWN, 0, "", "",option::Arg::Optional, "\nExamples:\n"
                               "  example --unknown -- --this_is_no_option\n"
                               "  example -unk --plus -ppp file1 file2\n" },
 {0,0,0,0,0,0}
};

struct MessageHeader {
    uint8_t comm_id{0};
    uint8_t reserved{0};
    uint16_t message_type{0};
    uint32_t time_seconds;
    int32_t time_nanoseconds;
};

static unsigned int old_heartbeat = 0;

bool debug_dump_message(const char *prefix_s, struct MessageHeader *pheader, unsigned char *message_body, int body_len)
{
  uint16_t seq;
  unsigned char *seq_offset = message_body + 4;
  switch (pheader->message_type) {
  case 0:    /* CommUartStats                  */
    //printf ("  %s: CommUartStats\n", prefix_s);
    break;
  case 1:    /* CommUartPing                   */
    //printf ("  %s: CommUartPing\n", prefix_s);
    break;
  case 2:    /* Heartbeat                      */
  {
    unsigned int my_heartbeat = *((unsigned int *)message_body);
#ifdef DEBUG_HEARTBEAT
    if ((my_heartbeat%1000)==0)
      printf ("  %s: Heartbeat : %d  (msg_len=%d)\n", prefix_s, my_heartbeat, body_len);
#endif

    if (my_heartbeat!=(old_heartbeat+100))
    {
        printf ("   Heartbeat annomaly? : %d (msg_len=%d)\n", my_heartbeat, body_len);
    }
    old_heartbeat = my_heartbeat;
    break;
  }
  case 3:     /* HeapStats                     */
    //printf ("  %s: HeapStats\n", prefix_s);
    break;
  case 4:     /* Reset                         */
    printf ("  %s: Reset\n", prefix_s);
    break;
  case 5:     /* GetNucleoFirmwareVersion      */
    printf ("  %s: GetNucleoFirmwareVersion\n", prefix_s);
    break;
  case 6:     /* TaskStats                     */
    printf ("  %s: TaskStats\n", prefix_s);
    break;
  case 7:     /* DbgTrace                      */
    printf ("  %s: DbgTrace\n", prefix_s);
    break;

  case 10:    /* MatchTimer                    */
    //printf ("  %s: MatchTimer\n", prefix_s);
    break;
  case 11:    /* MatchTimerStart               */
    printf ("  %s: MatchTimerStart\n", prefix_s);
    break;
  case 12:    /* MatchEnd                      */
    printf ("  %s: MatchEnd\n", prefix_s);
    break;
  case 13:    /* MatchTimerStop                */
    printf ("  %s: MatchTimerStop\n", prefix_s);
    break;

  case 20:    /* DbgGpioGet                    */
    printf ("  %s: DbgGpioGet\n", prefix_s);
    break;
  case 21:    /* DbgGpioGetStatus              */
    printf ("  %s: DbgGpioGetStatus\n", prefix_s);
    break;
  case 22:    /* DbgGpioSet                    */
    printf ("  %s: DbgGpioSet\n", prefix_s);
    break;
  case 23:    /* DbgPwmSet                     */
    printf ("  %s: DbgPwmSet\n", prefix_s);
    break;
  case 24:    /* DbgSetStatus                  */
    printf ("  %s: DbgSetStatus\n", prefix_s);
    break;

  case 29:    /* DbgGoldo                      */
    printf ("  %s: DbgGoldo\n", prefix_s);
    break;

  case 30:    /* FpgaReadReg                   */
    printf ("  %s: FpgaReadReg\n", prefix_s);
    break;
  case 31:    /* FpgaReadRegStatus             */
    printf ("  %s: FpgaReadRegStatus\n", prefix_s);
    break;
  case 32:    /* FpgaWriteReg                  */
    printf ("  %s: FpgaWriteReg\n", prefix_s);
    break;
  case 33:    /* SensorsState                  */
    //printf ("  %s: SensorsState\n", prefix_s);
    break;
  case 34:    /* FpgaGpioState                 */
    printf ("  %s: FpgaGpioState\n", prefix_s);
    break;
  case 35:    /* FpgaReadRegInternal           */
    printf ("  %s: FpgaReadRegInternal\n", prefix_s);
    break;
  case 36: /* FpgaReadAdc                      */
#ifdef DEBUG_ADC
    seq = *((uint16_t *)message_body);
    printf ("  %s: FpgaReadAdc\n", prefix_s);
#endif
    break;
  case 37: /* FpgaReadAdcOut                   */
#ifdef DEBUG_ADC
    seq = *((uint16_t *)message_body);
    printf ("  %s: FpgaReadAdcOut\n", prefix_s);
#endif
    break;

  case 40: /* ServoAck                         */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoAck seq=%d\n", prefix_s, seq);
    break;
  case 41: /* ServoMoveMultiple                */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoMoveMultiple seq=%d\n", prefix_s, seq);
    break;
  case 42: /* ServoSetEnable                   */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoSetEnable seq=%d\n", prefix_s, seq);
    break;
  case 43: /* ServoState                       */
    printf ("  %s: ServoState\n", prefix_s);
    break;
  case 44: /* ServosMoving                     */
    seq = *((uint16_t *)message_body);
    //printf ("  %s: ServosMoving seq=%d\n", prefix_s, seq);
    break;
  case 45: /* ServoDisableAll                  */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoDisableAll seq=%d\n", prefix_s, seq);
    break;
  case 46: /* ServoLiftsCmdRawObsolete         */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoLiftsCmdRawObsolete seq=%d\n", prefix_s, seq);
    break;
  case 47: /* ServoGetState                    */
    printf ("  %s: ServoGetState\n", prefix_s);
    break;
  case 48: /* ServoLiftDoHomingObsolete        */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoLiftDoHomingObsolete seq=%d\n", prefix_s, seq);
    break;
  case 49: /* ServoSetMaxTorques               */
    seq = *((uint16_t *)message_body);
    printf ("  %s: ServoSetMaxTorques seq=%d\n", prefix_s, seq);
    break;

  case 50: /* ODriveRequestPacket              */
    printf ("  %s: ODriveRequestPacket\n", prefix_s);
    break;
  case 51: /* ODriveResponsePacket             */
    printf ("  %s: ODriveResponsePacket\n", prefix_s);
    break;
  case 52: /* ODriveTelemetry                  */
    printf ("  %s: ODriveTelemetry\n", prefix_s);
    break;
  case 53: /* ODriveCommStats                  */
    printf ("  %s: ODriveCommStats\n", prefix_s);
    break;

  case 60: /* DynamixelsRequest                */
    printf ("  %s: DynamixelsRequest\n", prefix_s);
    break;
  case 61: /* DynamixelsResponse               */
    printf ("  %s: DynamixelsResponse\n", prefix_s);
    break;

  case 100: /* PropulsionEnableSet             */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionEnableSet seq=%d\n", prefix_s, seq);
    break;
  case 101: /* PropulsionMotorsEnableSet       */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionMotorsEnableSet seq=%d\n", prefix_s, seq);
    break;
  case 104: /* PropulsionSetAccelerationLimits */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionSetAccelerationLimits seq=%d\n", prefix_s, seq);
    break;
  case 105: /* PropulsionSetPose               */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionSetPose seq=%d\n", prefix_s, seq);
    break;
  case 108: /* PropulsionClearError            */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionClearError seq=%d\n", prefix_s, seq);
    break;
  case 110: /* PropulsionSetSimulationMode     */
    printf ("  %s: PropulsionSetSimulationMode\n", prefix_s);
    break;
  case 130: /* PropulsionCommandEvent          */
    seq = *((uint16_t *)seq_offset);
    printf ("\n----\n");
    printf ("  %s: PropulsionCommandEvent seq=%d\n", prefix_s, seq);
    printf ("\n----\n");
    break;
  case 131: /* PropulsionControllerEvent       */
    printf ("\n----\n");
    printf ("  %s: PropulsionControllerEvent\n", prefix_s);
    printf ("    message_body:\n");
    for (int i=0; i<32; i++)
    {
      printf (" %.2x", message_body[i]);
    }
    printf ("\n----\n");
    break;
  case 140: /* PropulsionExecuteTranslation    */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecuteTranslation seq=%d\n", prefix_s, seq);
    break;
  case 141: /* PropulsionExecuteMoveTo         */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecuteMoveTo seq=%d\n", prefix_s, seq);
    break;
  case 143: /* PropulsionExecutePointTo        */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecutePointTo seq=%d\n", prefix_s, seq);
    break;
  case 144: /* PropulsionExecuteFaceDirection  */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecuteFaceDirection seq=%d\n", prefix_s, seq);
    break;
  case 145: /* PropulsionExecuteTrajectory    */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecuteTrajectory seq=%d\n", prefix_s, seq);
    break;
  case 149: /* PropulsionExecuteReposition    */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecuteReposition seq=%d\n", prefix_s, seq);
    break;
  case 152: /* PropulsionODriveClearErrors     */
    printf ("  %s: PropulsionODriveClearErrors\n", prefix_s);
    break;
  case 153: /* PropulsionExecutePointToBack    */
    seq = *((uint16_t *)message_body);
    printf ("  %s: PropulsionExecutePointToBack seq=%d\n", prefix_s, seq);
    break;
  case 200: /* RobotConfigLoadBegin            */
    printf ("  %s: RobotConfigLoadBegin\n", prefix_s);
    break;
  case 201: /* RobotConfigLoadChunk            */
    printf ("  %s: RobotConfigLoadChunk\n", prefix_s);
    break;
  case 202: /* RobotConfigLoadEnd              */
    printf ("  %s: RobotConfigLoadEnd\n", prefix_s);
    break;
  case 230: /* LiftHomingDone                  */
    seq = *((uint16_t *)message_body);
    printf ("  %s: LiftHomingDone seq=%d\n", prefix_s, seq);
    break;
  case 231: /* LiftSetEnable                   */
    seq = *((uint16_t *)message_body);
    printf ("  %s: LiftSetEnable seq=%d\n", prefix_s, seq);
    break;
  case 232: /* LiftDoHoming                    */
    seq = *((uint16_t *)message_body);
    printf ("  %s: LiftDoHoming seq=%d\n", prefix_s, seq);
    break;
  case 233: /* LiftsCmdRaw                     */
    seq = *((uint16_t *)message_body);
    printf ("  %s: LiftsCmdRaw seq=%d\n", prefix_s, seq);
    break;
  default:
#ifdef DEBUG_FULL
    printf ("\n");
    printf ("  %s: ?? (%d)\n", prefix_s, pheader->message_type);
    printf ("    message_body:\n");
    for (int i=0; i<32; i++)
    {
      printf (" %.2x", message_body[i]);
    }
    printf ("\n");
    break;
#endif
    return false;
  }
  return true;
}

int main(int argc, char *argv[])
{
    bool debug_flag = false;

    argc-=(argc>0); argv+=(argc>0); // skip program name argv[0] if present

    option::Stats  stats(usage, argc, argv);
    option::Option options[stats.options_max], buffer[stats.buffer_max];
    option::Parser parse(usage, argc, argv, options, buffer);

    uint8_t comm_id{0};
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

    if (options[COMM_ID])
    {
        comm_id = std::atoi(options[COMM_ID].arg);
    };

    if (options[DEBUG])
    {
        debug_flag = true;
        printf ("DEBUG\n");
    };

    std::cout << "PUB port: " << pub_port << " SUB port: " << sub_port << "comm id: " << comm_id << "\n";
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

    init_zmq(pub_port, sub_port, comm_id);

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

            struct timespec tv;
            clock_gettime(CLOCK_MONOTONIC, &tv);

            while(deserializer.message_ready())
            {
                uint16_t recv_message_type = deserializer.message_type();
                size_t recv_message_size = deserializer.message_size();
                deserializer.pop_message(tmp_buffer, sizeof(tmp_buffer));

                MessageHeader header;
                header.comm_id = comm_id;
                header.message_type = recv_message_type;
                header.time_seconds = tv.tv_sec;
                header.time_nanoseconds = tv.tv_nsec;

                if (debug_flag)
                {
                    debug_dump_message("R<<<<N", &header, tmp_buffer, recv_message_size);
                }

                zmq_send(pub_socket, (const char*)(&header), sizeof(header), ZMQ_SNDMORE);
                zmq_send(pub_socket, (const char*)(tmp_buffer), recv_message_size, 0);
            }
        }

        // Read message from zmq
        if(poll_items[1].revents && ZMQ_POLLIN)
        {
            unsigned char buff[1024];
            MessageHeader header;
            size_t bytes_read = 0;
            int64_t more=1;
            size_t more_size = sizeof(more);
            while(more)
            {
                bytes_read += zmq_recv(sub_socket, buff + bytes_read, sizeof(buff) - bytes_read, 0);
                zmq_getsockopt(sub_socket, ZMQ_RCVMORE, &more, &more_size);
            }
            buff[bytes_read] = 0;
            if(bytes_read >= sizeof(header))
            {
                memcpy(&header, buff, sizeof(header));
                serializer.push_message(header.message_type, buff+sizeof(header), bytes_read - sizeof(header));
            };

            if (debug_flag)
            {
                unsigned char *message_body = buff + sizeof(MessageHeader);
                uint16_t seq;
#ifndef DEBUG_FULL
                if ((header.message_type!=0) && (header.message_type!=1) && (header.message_type!=36))
#endif
                {
                    printf ("\n");
                    printf ("header.message_type=%d bytes_read=%d\n", header.message_type, (int)bytes_read);
                    for (int i=0; i<32; i++)
                    {
                        printf (" %.2x", buff[i]);
                    }
                    printf ("\n");

                    if (!debug_dump_message("R>>>>N", &header, message_body, bytes_read))
                    {
                        printf ("  R>>>>N: received UNKNOWN command code : %x (%d)\n",
                                header.message_type, header.message_type);
                    }

                    printf ("\n");
                }
            }

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

