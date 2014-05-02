#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>
#include <stdint.h>
#define WIFLY_PORT "2000"
#define WIFLY_ADDR "169.254.1.1"
#define BUF_SIZE 64
#define SOCKET_OFF 0
#define SOCKET_ON 1
#define TIMEOUT_MAX_RETRY 5
#define MASTER_COMMAND_TRANSMISSION_START 31
#define SLAVE_COMMAND_ACK 30
#define MASTER_COMMAND_TOGGLE_SOCKET 29
#define MASTER_COMMAND_REQUEST_SOCKET_STATUS 28
#define MASTER_COMMAND_SET_TIME 27
#define MASTER_COMMAND_ENERGY_QUERY 26
#define PRINT_USAGE_AND_CONTINUE() {print_usage();continue;}
#define CLEAR_CMD_BUF() memset(cmd_buf, 0, BUF_SIZE)
#define CLEAR_SEND_BUF() memset(send_buf, 0, BUF_SIZE)
#define CLEAR_RECV_BUF() memset(recv_buf, 0, BUF_SIZE)
#define TIMEOUT_SECONDS 15
#define ONE_DAY_IN_SEC 86400
#define ONE_HOUR_IN_SEC 3600
#define KWH_IN_J 3600000
#define CENT_PER_KWH 9
#define MAINS_VOLTAGE_RMS 120

char cmd_buf[BUF_SIZE];
uint8_t send_buf[BUF_SIZE];
uint8_t recv_buf[BUF_SIZE];
int32_t sockfd;

void do_command();
int32_t recv_from_client(uint8_t *buf);
void send_to_client(uint8_t *buf, int32_t len);
void print_usage();
void send_cmd_toggle_socket(int32_t socket_num, int32_t socket_state);
void wifi_init();
void send_cmd_request_socket_status();
int32_t recv_one_byte(uint8_t* c, int32_t timeout);
void send_cmd_set_time();
void int32_to_char(long int32, uint8_t *c);
void int16_to_char(int32_t int16, uint8_t *c);
int32_t char_to_int16(uint8_t* c);
long char_to_int32(uint8_t* c);
void send_cmd_energy_query(time_t start_utc, time_t end_utc);
int32_t is_number(char c);
void flush_recv_buf(int32_t timeout);

void flush_recv_buf(int32_t timeout)
{
    uint8_t c;
    int32_t count = 0;
    while(recv_one_byte(&c, timeout) == 0)
        count++;
    count > 0 ? printf("flushed %d bytes\n", count) : count;
}

int32_t is_number(char c)
{
    return c >= '0' && c <= '9';
}

// from example code in Beej's Guide to Network Programming
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) 
        return &(((struct sockaddr_in*)sa)->sin_addr);
    
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

// from example code in Beej's Guide to Network Programming
int32_t main(int32_t argc, char *argv[])
{
    struct addrinfo hints, *servinfo, *p;
    int32_t rv;
    char s[INET6_ADDRSTRLEN];
    srand(time(NULL));
    char wifly_address[20];
    
    if(argc > 2) 
    {
        fprintf(stderr,"usage: 445_PC, 445_PC addr\n");
        exit(1);
    }

    if(argc == 2)
        strcpy(wifly_address, argv[1]);
    else
        strcpy(wifly_address, WIFLY_ADDR);

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if ((rv = getaddrinfo(wifly_address, WIFLY_PORT, &hints, &servinfo)) != 0) 
    {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    for(p = servinfo; p != NULL; p = p->ai_next) 
    {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) 
        {
            perror("445_PC: socket");
            continue;
        }

        if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) 
        {
            close(sockfd);
            perror("445_PC: connect");
            continue;
        }
        break;
    }

    if (p == NULL) 
    {
        fprintf(stderr, "445_PC: failed to connect\n");
        return 2;
    }

    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr),
            s, sizeof s);
    printf("445_PC: connected to %s\n", s);

    freeaddrinfo(servinfo);
    do_command(sockfd);
    close(sockfd);
    return 0;
}

// flush out WiFi module's greeting, as well as any
// remaining bytes in the sockets
void wifi_init()
{
    printf("Flushing receive buffer...\n");
    fcntl(sockfd, F_SETFL, O_NONBLOCK); // set socket to non-block
    flush_recv_buf(3);
    CLEAR_RECV_BUF();
    printf("Ready.\n");
}

void do_command()
{
    wifi_init();
    // main command line loop
    while(1)
    {
        CLEAR_CMD_BUF();
        CLEAR_SEND_BUF();
        printf("$: ");
        // get user's command and put it in cmd_buf
        fgets(cmd_buf, BUF_SIZE-1, stdin);

        // quit
        if(strcmp(cmd_buf, "q\n") == 0)
            return;

        // toggle socket.
        else if((cmd_buf[0] == 's') && (cmd_buf[1] <= '3' && cmd_buf[1] >= '1'))
        {
            if(strcmp(cmd_buf+2, "1\n") == 0)
                send_cmd_toggle_socket(cmd_buf[1], SOCKET_ON);

            else if(strcmp(cmd_buf+2, "0\n") == 0)
                send_cmd_toggle_socket(cmd_buf[1], SOCKET_OFF);

            else
                PRINT_USAGE_AND_CONTINUE();
        }
        // socket state
        else if(strcmp(cmd_buf, "ss\n") == 0)
            send_cmd_request_socket_status();
        // all on
        else if(strcmp(cmd_buf, "a1\n") == 0)
        {
            for(char i = '1'; i <= '4'; i ++)
                send_cmd_toggle_socket(i, SOCKET_ON);
        }
        // all off
        else if(strcmp(cmd_buf, "a0\n") == 0)
        {
            for(char i = '1'; i <= '4'; i++)
                send_cmd_toggle_socket(i, SOCKET_OFF);
        }
        // set time
        else if(strcmp(cmd_buf, "st\n") == 0)
            send_cmd_set_time();
        // calculate energy
        else if(cmd_buf[0] == 'e' && is_number(cmd_buf[1])) // e3h
        {
            int32_t duration = atoi(&cmd_buf[1]);
            if(duration == 0)
                PRINT_USAGE_AND_CONTINUE();

            // get the position of the first letter after numbers
            int32_t i = 1;
            for(; i < strlen(cmd_buf) - 1; i++)
                if(!is_number(cmd_buf[i]))
                    break;

            printf("energy used during the past %d ", duration);
            switch(cmd_buf[i])
            {
                case 'h':
                printf("hour");
                if(duration > 1)
                    printf("s");
                printf(":\n");
                send_cmd_energy_query(time(0) - ONE_HOUR_IN_SEC * duration, time(0));
                break;

                case 'd':
                printf("day");
                if(duration > 1)
                    printf("s");
                printf(":\n");
                send_cmd_energy_query(time(0) - ONE_DAY_IN_SEC * duration, time(0));
                break;

                case 'w':
                printf("week");
                if(duration > 1)
                    printf("s");
                printf(":\n");
                send_cmd_energy_query(time(0) - ONE_DAY_IN_SEC * 7 * duration, time(0));
                break;

                case 'm':
                printf("month");
                if(duration > 1)
                    printf("s");
                printf(":\n");
                send_cmd_energy_query(time(0) - ONE_DAY_IN_SEC * 30 * duration, time(0));
                break;

                case 'y':
                printf("year");
                if(duration > 1)
                    printf("s");
                printf(":\n");
                send_cmd_energy_query(time(0) - ONE_DAY_IN_SEC * 365 * duration, time(0));
                break;

                default:
                printf("\'%c\'? I don't know what that is.\n", cmd_buf[i]);
                printf("e#[h,d,w,m,y]: get energy usage for the last # hour/day/week/month/year\n");
                continue;
            }
        }
        // energy between two points in time
        // eg YYYY MM DD hr min sec YYYY MM DD hr min sec
        else if(strncmp(cmd_buf, "eq ", 3) == 0)
        {
            int32_t date_valus[12];
            for(int32_t i = 0, j = 0; i < strlen(cmd_buf) - 1; i++)
                if((cmd_buf[i] == ' ') && (cmd_buf[i + 1] != ' '))
                {
                    date_valus[j] = atol(&cmd_buf[i]);
                    if(++j >= 12)
                        break;
                }
            struct tm start_tm_local, end_tm_local;
            start_tm_local.tm_year = date_valus[0] - 1900;
            start_tm_local.tm_mon = date_valus[1] - 1;
            start_tm_local.tm_mday = date_valus[2];
            start_tm_local.tm_hour = date_valus[3];
            start_tm_local.tm_min = date_valus[4];
            start_tm_local.tm_sec = date_valus[5];
            start_tm_local.tm_isdst = -1;
            end_tm_local.tm_year = date_valus[6] - 1900;
            end_tm_local.tm_mon = date_valus[7] - 1;
            end_tm_local.tm_mday = date_valus[8];
            end_tm_local.tm_hour = date_valus[9];
            end_tm_local.tm_min = date_valus[10];
            end_tm_local.tm_sec = date_valus[11];
            end_tm_local.tm_isdst = -1;
            send_cmd_energy_query(mktime(&start_tm_local), mktime(&end_tm_local));
        }
        // debug command, flush recv_buf
        else if(strcmp(cmd_buf, "f\n") == 0)
            flush_recv_buf(1);
        else
            PRINT_USAGE_AND_CONTINUE();
    }
}

// ask power strip how much energy it has used between start_utc and
// end_utc, then print it out in kWh, as well as the cost in $.
void send_cmd_energy_query(time_t start_utc, time_t end_utc)
{
    // fill send_buf with command and two dates
    send_buf[0] = MASTER_COMMAND_ENERGY_QUERY;
    int32_to_char(start_utc, send_buf + 1);
    int32_to_char(end_utc, send_buf + 5);
    send_to_client(send_buf, 9);
    // now the result is in recv_buf
    uint32_t result[4];
    double total_kwh = 0;
    for(int32_t i = 0; i < 3; i++)
    {
        // get the result out of recv_buf and print it out
        result[i] = char_to_int32(recv_buf + i * 4);
        double kwh = (double)result[i] / KWH_IN_J;
        total_kwh += kwh;
        printf("Socket %d: %.4fkWh, $%.4f\n", i+1, kwh, kwh * CENT_PER_KWH / 100);
    }
    printf("   Total: %.4fkWh, $%.4f\n", total_kwh, total_kwh * CENT_PER_KWH / 100);
}

// send current time to set the RTC in power strip
void send_cmd_set_time()
{
    send_buf[0] = MASTER_COMMAND_SET_TIME;
    int32_to_char(time(0), send_buf+1);
    send_to_client(send_buf, 5);
}

// ask power strip the state of each socket
void send_cmd_request_socket_status()
{   
    // fill send_buf
    send_buf[0] = MASTER_COMMAND_REQUEST_SOCKET_STATUS;
    send_to_client(send_buf, 1);
    // now recv_buf has the result
    int32_t socket_current[4];
    // copy the result to socket_current
    for(int32_t i = 0; i < 4; i++)
        socket_current[i] = char_to_int16(&recv_buf[1+2*i]);
    // then print everything out
    for(int32_t i = 0; i < 3; i++)
    {
        printf("Socket %d: ", i + 1);
        if(recv_buf[0] & (1 << i))
            printf("ON");
        else
            printf("OFF");
        double current = (double)socket_current[i] / 1000;
        printf(", %.3fA, %.3fW\n", current, current * MAINS_VOLTAGE_RMS);
    }
}

// ask power strip to toggle socket 
void send_cmd_toggle_socket(int32_t socket_num, int32_t socket_state)
{
    send_buf[0] = MASTER_COMMAND_TOGGLE_SOCKET;
    send_buf[1] = socket_num - '1';
    send_buf[2] = socket_state;
    send_to_client(send_buf, 3);
}

// attach a header then send len bytes from start of buf to 
// the power strip, then wait for its response.
void send_to_client(uint8_t *buf, int32_t len)
{
    if(len <= 0 || len > 255)
    {
        printf("send to client invalid message length\n");
        return;
    }
    // assemble header
    uint8_t header[2];
    header[0] = MASTER_COMMAND_TRANSMISSION_START;
    header[1] = len;
    // retry if timeout happens
    for (int32_t i = 0; i < TIMEOUT_MAX_RETRY; i++)
    {
        if(i > 0)
            printf("send command timeout, retry #%d\n", i);
        // send out header
        if(send(sockfd, header, 2, 0) == -1)
        {
            perror("send");
            exit(0);
        }
        // send out data
        if(send(sockfd, buf, len, 0) == -1)
        {
            perror("send");
            exit(0);
        }
        // wait for response
        if(recv_from_client(recv_buf) == 0)
            return;
    }
    // if all retries result in timeout, exit the program
    printf("can not reach client\n");
    exit(0);
}

// listen to power strip's response, return -1 for fail, 0 for success
// result is stored in buf                               
int32_t recv_from_client(uint8_t *buf)
{
    int32_t message_length;
    uint8_t c;
    memset(buf, 0, BUF_SIZE);
    // discard incoming bytes until the start of slave's response
    do
    {
        if(recv_one_byte(&c, TIMEOUT_SECONDS) == -1)
            return -1;
    }
    while(c != SLAVE_COMMAND_ACK);

    // now we have received slave's ACK, the next byte is
    // slave's message length
    if(recv_one_byte(&c, TIMEOUT_SECONDS) == -1)
        return -1;
    message_length = c;
    // receive corresponding number of bytes and store them
    // in buf
    for (int32_t i = 0; i < message_length && i < BUF_SIZE; i++)
    {
        if(recv_one_byte(&c, TIMEOUT_SECONDS) == -1)
            return -1;
        buf[i] = c;
    }
    return 0;
}

// get one byte from socket, if nothing available
// return -1 afer timeout seconds
int32_t recv_one_byte(uint8_t* c, int32_t timeout)
{
    uint8_t byte_buf[1];
    time_t now = time(0);
    while(time(0) - now < timeout)
        if(recv(sockfd, byte_buf, 1, 0) != -1)
        {
            *c = byte_buf[0];
            return 0;
        }
    return -1;
}

void print_usage()
{
    printf("\n");
    printf("usage:\n");
    printf("s[1,2,3][0,1]:      toggle socket. s10 turns socket 1 off, s21 turns socket 2 on, etc.\n");
    printf("a1:                 turn on all sockets\n");
    printf("a0:                 turn off all sockets\n");
    printf("ss:                 get socket status\n");
    printf("e#[h,d,w,m,y]:      get energy usage for the past # hour/day/week/month/year\n");
    printf("eq YYYY MM DD HH MM SS YYYY MM DD HH MM SS:\n");
    printf("                    get energy query between two timestamps\n");
    printf("st:                 set power strip's RTC\n");
    printf("q:                  quit\n");
    printf("\n");
}

// fill the byte array with each byte in an int32_t, little endian
void int32_to_char(long int32, uint8_t *c)
{
    c[0] = int32 & 0xff;
    c[1] = (int32 & 0xff00) >> 8;
    c[2] = (int32 & 0xff0000) >> 16;
    c[3] = (int32 & 0xff000000) >> 24;
}

// fill the byte array with each byte in an int16_t, little endian
void int16_to_char(int32_t int16, uint8_t *c)
{
    c[0] = int16 & 0xff;
    c[1] = (int16 & 0xff00) >> 8;
}

// extract an int16_t from a byte array, little endian
int32_t char_to_int16(uint8_t* c)
{
    int32_t ret = 0;
    ret = c[0];
    ret |= c[1] << 8;
    return ret;
}

// extract an int32_t from a byte array, little endian
long char_to_int32(uint8_t* c)
{
    long ret = 0;
    ret = c[0];
    ret |= c[1] << 8;
    ret |= c[2] << 16;
    ret |= c[3] << 24;
    return ret;
}