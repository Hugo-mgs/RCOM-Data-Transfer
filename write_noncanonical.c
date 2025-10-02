// Example of how to write to the serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 5

int fd = -1;           // File descriptor for open serial port
struct termios oldtio; // Serial port settings to restore on closing
volatile int STOP = FALSE;

// Create string to send
unsigned char buf[BUF_SIZE] = {0};

int alarmEnabled = FALSE;
int alarmCount = 0;

int openSerialPort(const char *serialPort, int baudRate);
int closeSerialPort();
int readByteSerialPort(unsigned char *byte);
int writeBytesSerialPort(const unsigned char *bytes, int nBytes);

// ---------------------------------------------------
// ALARM HANDLER
// ---------------------------------------------------
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);

    printf("Alarm #%d received\n", alarmCount);
    printf("%d bytes written to serial port\n", bytes);
}

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS0\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    //
    // NOTE: See the implementation of the serial port library in "serial_port/".
    const char *serialPort = argv[1];

    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }

    printf("Serial port %s opened\n", serialPort);

    // Configure the alarm

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }

    printf("Alarm configured\n");

    unsigned char flag = 0x7E;
    unsigned char address = 0x03;
    unsigned char control = 0x03;
    unsigned char bcc = address ^ control;

    buf[0] = flag;
    buf[1] = address;
    buf[2] = control;
    buf[3] = bcc;
    buf[4] = flag;

    int bytes = writeBytesSerialPort(buf, BUF_SIZE);
    printf("%d bytes written to serial port\n", bytes);

    alarm(3);

    int nBytesBuf = 0;
    int state = 0;
    unsigned char bcc_receive;
    while (STOP == FALSE && alarmCount < 4)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;
        }

        // Read one byte from serial port.
        // NOTE: You must check how many bytes were actually read by reading the return value.
        // In this example, we assume that the byte is always read, which may not be true.
        unsigned char byte;
        int bytes = readByteSerialPort(&byte);

        if (bytes == 0) continue;
        printf("var = 0x%02X ", byte);
        printf("state = %d\n", state);
        
        nBytesBuf += bytes;

        if(byte == 0x7e && state != 4) state = 1;
        else if((byte == 0x03) && state == 1) {
            state++;
            bcc_receive = byte;
        }
        else if (byte == 0x07 && state == 2) {
            state++;
            bcc_receive = bcc_receive ^ byte;
        }
        else if (byte == bcc_receive && state == 3) {
            state++;
            printf ("Correct guy\n");
        }
        else if (byte == 0x7e && state == 4) {
            STOP = TRUE;
        }
        else state = 0;
    }
    if (alarmCount >= 4) {
        printf("Gave up after %d alarms\n", alarmCount);
    } else {
        alarm(0);
    }

    // Wait until all bytes have been written to the serial port
    sleep(1);

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }

    printf("Serial port %s closed\n", serialPort);

    return 0;
}

// ---------------------------------------------------
// SERIAL PORT LIBRARY IMPLEMENTATION
// ---------------------------------------------------

// Open and configure the serial port.
// Returns -1 on error.
int openSerialPort(const char *serialPort, int baudRate)
{
    // Open with O_NONBLOCK to avoid hanging when CLOCAL
    // is not yet set on the serial port (changed later)
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd = open(serialPort, oflags);
    if (fd < 0)
    {
        perror(serialPort);
        return -1;
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    // Convert baud rate to appropriate flag

    // Baudrate settings are defined in <asm/termbits.h>, which is included by <termios.h>
#define CASE_BAUDRATE(baudrate) \
    case baudrate:              \
        br = B##baudrate;       \
        break;

    tcflag_t br;
    switch (baudRate)
    {
        CASE_BAUDRATE(1200);
        CASE_BAUDRATE(1800);
        CASE_BAUDRATE(2400);
        CASE_BAUDRATE(4800);
        CASE_BAUDRATE(9600);
        CASE_BAUDRATE(19200);
        CASE_BAUDRATE(38400);
        CASE_BAUDRATE(57600);
        CASE_BAUDRATE(115200);
    default:
        fprintf(stderr, "Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n");
        return -1;
    }
#undef CASE_BAUDRATE

    // New port settings
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = br | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 1; // Block reading
    newtio.c_cc[VMIN] = 0;  // Byte by byte

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    // Clear O_NONBLOCK flag to ensure blocking reads
    oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1)
    {
        perror("fcntl");
        close(fd);
        return -1;
    }

    return fd;
}

// Restore original port settings and close the serial port.
// Returns 0 on success and -1 on error.
int closeSerialPort()
{
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    return close(fd);
}

// Wait up to 0.1 second (VTIME) for a byte received from the serial port.
// Must check whether a byte was actually received from the return value.
// Save the received byte in the "byte" pointer.
// Returns -1 on error, 0 if no byte was received, 1 if a byte was received.
int readByteSerialPort(unsigned char *byte)
{
    return read(fd, byte, 1);
}

// Write up to numBytes from the "bytes" array to the serial port.
// Must check how many were actually written in the return value.
// Returns -1 on error, otherwise the number of bytes written.
int writeBytesSerialPort(const unsigned char *bytes, int nBytes)
{
    return write(fd, bytes, nBytes);
}
