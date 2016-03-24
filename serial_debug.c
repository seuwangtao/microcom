#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <assert.h>

#define LOCK_FILE "/var/lock/serial"
#define DEFAULT_DEVICE "/dev/ttyS2"
#define DEFAULT_BAUDRATE 115200
#define BUFSIZE 1024

static struct termios pots;             /* old port termios settings to restore */
static struct termios sots;         /* old stdout/in termios settings to restore */
int fd;

static void init_comm(struct termios *pts)
{
    /* some things we want to set arbitrarily */
    pts->c_lflag &= ~ICANON;
    pts->c_lflag &= ~(ECHO | ECHOCTL | ECHONL);
    pts->c_cflag |= HUPCL;
    pts->c_cflag |= CS8;
    pts->c_iflag |= IGNBRK;
    pts->c_cc[VMIN] = 1;
    pts->c_cc[VTIME] = 0;

    /* Standard CR/LF handling: this is a dumb terminal.
     * Do no translation:
     *  no NL -> CR/NL mapping on output, and
     *  no CR -> NL mapping on input.
     */
    pts->c_oflag &= ~ONLCR;
    pts->c_iflag &= ~ICRNL;
    
    /* no flow control */
    pts->c_cflag &= ~CRTSCTS;
    pts->c_iflag &= ~(IXON | IXOFF | IXANY);

}

static int baudrate_to_flag(int speed, speed_t *flag)
{
        switch(speed) {
        case 50: *flag = B50; return 0;
        case 75: *flag = B75; return 0;
        case 110: *flag = B110; return 0;
        case 134: *flag = B134; return 0;
        case 150: *flag = B150; return 0;
        case 200: *flag = B200; return 0;
        case 300: *flag = B300; return 0;
        case 600: *flag = B600; return 0;
        case 1200: *flag = B1200; return 0;
        case 1800: *flag = B1800; return 0;
        case 2400: *flag = B2400; return 0;
        case 4800: *flag = B4800; return 0;
        case 9600: *flag = B9600; return 0;
        case 19200: *flag = B19200; return 0;
        case 38400: *flag = B38400; return 0;
        case 57600: *flag = B57600; return 0;
        case 115200: *flag = B115200; return 0;
       default:
            printf("unknown speed: %d\n",speed);
            return -1;
       }
}

static int serial_set_speed(int fd, speed_t speed)
{
        struct termios pts;     /* termios settings on port */

        tcgetattr(fd, &pts);

        cfsetospeed(&pts, speed);
        cfsetispeed(&pts, speed);
        tcsetattr(fd, TCSANOW, &pts);

        return 0;
}

static void init_terminal(void)
{
        struct termios sts;

        memcpy(&sts, &sots, sizeof (sots));     /* to be used upon exit */

        /* again, some arbitrary things */
        sts.c_iflag &= ~(IGNCR | INLCR | ICRNL);
        sts.c_iflag |= IGNBRK;
        sts.c_lflag &= ~ISIG;
        sts.c_cc[VMIN] = 1;
        sts.c_cc[VTIME] = 0;
        sts.c_lflag &= ~ICANON;
        /* no local echo: allow the other end to do the echoing */
        sts.c_lflag &= ~(ECHO | ECHOCTL | ECHONL);

        tcsetattr(STDIN_FILENO, TCSANOW, &sts);
}

static void handle_exit(int signal)
{
        printf("exiting\n");

        tcsetattr(fd, TCSANOW, &pots);
        close(fd);

        tcsetattr(STDIN_FILENO, TCSANOW, &sots);
        exit(0);
}

static void write_receive_buf(const unsigned char *buf, int len)
{
        if (!len)
                return;

        write(STDOUT_FILENO, buf, len);
}

static int do_commandline(void)
{
        int ret;

        tcsetattr(fd, TCSANOW, &pots);
//      close(fd);

        tcsetattr(STDIN_FILENO, TCSANOW, &sots);

        return 0;
}


/* handle escape characters, writing to output */
static void cook_buf(int fd, unsigned char *buf, int num)
{
        int current = 0;

        while (current < num) { /* big while loop, to process all the charactes in buffer */

                /* look for the next escape character (Ctrl-\) */
                while ((current < num) && (buf[current] != 28))
                        current++;
                /* and write the sequence before esc char to the comm port */
                if (current)
                        write(fd, buf, current);

                if (current < num) {    /* process an escape sequence */
                        /* found an escape character */
                        do_commandline();
                        return;
                }               /* if - end of processing escape sequence */
                num -= current;
                buf += current;
                current = 0;
        }                       /* while - end of processing all the charactes in the buffer */
}


/* main program loop */
static int mux_loop(void)
{
        fd_set ready;           /* used for select */
        int i = 0, len;         /* used in the multiplex loop */
        unsigned char buf[BUFSIZE];

        while (1) {
                FD_ZERO(&ready);
                FD_SET(STDIN_FILENO, &ready);
                FD_SET(fd, &ready);

                select(fd + 1, &ready, NULL, NULL, NULL);

                if (FD_ISSET(fd, &ready)) {
                        /* pf has characters for us */
                        len = read(fd, buf, BUFSIZE);
                        if (len < 0)
                                return -errno;
                        if (len == 0)
                                return -EINVAL;

                        write(STDOUT_FILENO, buf, len);
                }

                if (FD_ISSET(STDIN_FILENO, &ready)) {
                        /* standard input has characters for us */
                        i = read(STDIN_FILENO, buf, BUFSIZE);
                        if (i < 0)
                                return -errno;
                        if (i == 0)
                                return -EINVAL;

                        cook_buf(fd, buf, i);
                }
        }
}


int main(void)
{
    char *device = DEFAULT_DEVICE;
    char *lockfile = LOCK_FILE; 
        struct termios pts;
    speed_t flag;
    int current_speed = DEFAULT_BAUDRATE;
        int lockfd, ret;
        struct sigaction sact;  /* used to initialize the signal handler */

        lockfd = open(lockfile, O_RDONLY);
        if (lockfd > 0) {
                close(fd);
                printf("lockfile exists, cannot open device %s\n", device);
                exit(1);
        }

        lockfd = open(lockfile, O_RDONLY | O_CREAT);
        if (fd < 0 ) {
                printf("cannot create lockfile. ignoring\n");
                exit(1);
        }

    fd = open(device, O_RDWR);
    if (fd < 0) {
        printf("cannot open device %s", device);
                exit(1);
        }
    
    /* modify the port configuration */
    tcgetattr(fd, &pts);
    memcpy(&pots, &pts, sizeof (pots));
    init_comm(&pts);
    tcsetattr(fd, TCSANOW, &pts);
    printf("connected to %s\n", device);

    baudrate_to_flag(current_speed, &flag);
    serial_set_speed(fd, flag);

    printf("Escape character: Ctrl-\\\n");

    /* Now deal with the local terminal side */
    tcgetattr(STDIN_FILENO, &sots);
    init_terminal();

    /* set the signal handler to restore the old
     * termios handler */
    sact.sa_handler = &handle_exit;
    sigaction(SIGHUP, &sact, NULL);
    sigaction(SIGINT, &sact, NULL);
    sigaction(SIGPIPE, &sact, NULL);
    sigaction(SIGTERM, &sact, NULL);
    sigaction(SIGQUIT, &sact, NULL);

    /* run the main program loop */
    ret = mux_loop();
    if (ret)
        fprintf(stderr, "%s\n", strerror(-ret));

    tcsetattr(fd, TCSANOW, &pots);
    close(fd);
    tcsetattr(STDIN_FILENO, TCSANOW, &sots);

    exit(ret ? 1 : 0);
}
