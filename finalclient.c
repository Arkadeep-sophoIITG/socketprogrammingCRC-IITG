#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <limits.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>

typedef struct packet {
    char data[1024];
} Packet;

typedef struct frame {
    int frame_kind; //ACK:0, SEQ:1 FIN:2
    int sq_no;
    int ack;
    Packet packet;
} Frame;

void error(char *msg) {
    perror(msg);
    exit(0);
}



// CRC parameters (default values are for CRC-8):

const int order = 8;
const unsigned long polynom = 0x07;
const int direct = 1;
const unsigned long crcinit = 0x00;
const unsigned long crcxor = 0x00;
const int refin = 0;
const int refout = 0;

// 'order' [1..32] is the CRC polynom order, counted without the leading '1' bit
// 'polynom' is the CRC polynom without leading '1' bit
// 'direct' [0,1] specifies the kind of algorithm: 1=direct, no augmented zero bits
// 'crcinit' is the initial CRC value belonging to that algorithm
// 'crcxor' is the final XOR value
// 'refin' [0,1] specifies if a data byte is reflected before processing (UART) or not
// 'refout' [0,1] specifies if the CRC will be reflected before XOR



// internal global values:

unsigned long crcmask;
unsigned long crchighbit;
unsigned long crcinit_direct;
unsigned long crcinit_nondirect;
unsigned long crctab[256];


unsigned long reflect(unsigned long crc, int bitnum) {

    // reflects the lower 'bitnum' bits of 'crc'

    unsigned long i, j = 1, crcout = 0;

    for (i = (unsigned long) 1 << (bitnum - 1); i; i >>= 1) {
        if (crc & i) crcout |= j;
        j <<= 1;
    }
    return (crcout);
}


void generate_crc_table() {

    // make CRC lookup table used by table algorithms

    int i, j;
    unsigned long bit, crc;

    for (i = 0; i < 256; i++) {

        crc = (unsigned long) i;
        if (refin) crc = reflect(crc, 8);
        crc <<= order - 8;

        for (j = 0; j < 8; j++) {

            bit = crc & crchighbit;
            crc <<= 1;
            if (bit) crc ^= polynom;
        }

        if (refin) crc = reflect(crc, order);
        crc &= crcmask;
        crctab[i] = crc;
    }
}

unsigned long crctablefast(unsigned char *p, unsigned long len) {

    // fast lookup table algorithm without augmented zero bytes, e.g. used in pkzip.
    // only usable with polynom orders of 8, 16, 24 or 32.

    unsigned long crc = crcinit_direct;

    if (refin) crc = reflect(crc, order);

    if (!refin) while (len--) crc = (crc << 8) ^ crctab[((crc >> (order - 8)) & 0xff) ^ *p++];
    else while (len--) crc = (crc >> 8) ^ crctab[(crc & 0xff) ^ *p++];

    if (refout ^ refin) crc = reflect(crc, order);
    crc ^= crcxor;
    crc &= crcmask;

    return (crc);
}

char *stringToBinary(char *s) {
    if (s == NULL) {
        // NULL might be 0 but you cannot be sure about it
        return NULL;
    }
    // get length of string without NUL
    size_t slen = strlen(s);

    // we cannot do that here, why?
    // if(slen == 0){ return s;}

    errno = 0;
    // allocate "slen" (number of characters in string without NUL)
    // times the number of bits in a "char" plus one byte for the NUL
    // at the end of the return value
    char *binary = malloc(slen * CHAR_BIT + 1);
    if (binary == NULL) {
        fprintf(stderr, "malloc has -1ed in stringToBinary(%s): %s\n", s, strerror(errno));
        return NULL;
    }
    // finally we can put our shortcut from above here
    if (slen == 0) {
        *binary = '\0';
        return binary;
    }
    char *ptr;
    // keep an eye on the beginning
    char *start = binary;
    int i;

    // loop over the input-characters
    for (ptr = s; *ptr != '\0'; ptr++) {
        /* perform bitwise AND for every bit of the character */
        // loop over the input-character bits
        for (i = CHAR_BIT - 1; i >= 0; i--, binary++) {
            *binary = (*ptr & 1 << i) ? '1' : '0';
        }
    }
    // finalize return value
    *binary = '\0';
    // reset pointer to beginning
    binary = start;
    return binary;
}


void random_flip(char *rem_message, float BER) {
    unsigned int i = 0;
    srand(time(0));
    for (i = 0; i < strlen(rem_message); i++) {
        if ((float) rand() / RAND_MAX < BER) {
            if (rem_message[i] == '0') {
                rem_message[i] = '1';
            } else if (rem_message[i] == '1') {
                rem_message[i] = '0';
            } else {
                fprintf(stderr, "Convert to binary was not successful\n");
            }
        }
    }
}

int connection_setup(int port, char *addr) {

    struct sockaddr_in serv_addr;
    int socket_client; // client fd
    // create a new socket witch AF_INET for internet domain, stream socket option, TCP(given by os) - reliable, connection oriented
    if ((socket_client = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        error("Error! failed to open the socket\n");
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port); // convert a port number in host byte order to a port number in network byte order
    serv_addr.sin_addr.s_addr = inet_addr(addr);
    bzero(&serv_addr.sin_zero, 8); // clears the buffer

    struct timeval tv;
    tv.tv_sec = 5;  // 5 Secs timeout
    tv.tv_usec = 0; // not initialising this can cause strange errors

    // set socket option for timeout
    setsockopt(socket_client, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval));

    // connect the socket referred by the fd to the addr specified by socket addr
    if (connect(socket_client, (struct sockaddr *) &serv_addr, sizeof(struct sockaddr_in)) < 0) {
        error("ERROR connecting");
    }
    return socket_client;
}


int server_communicate(int socket_client, char *formed_message) {

    int str_len;
    socklen_t adr_sz;
    struct sockaddr_in serv_adr, from_adr;
    Frame ackframe, recv_frame;
    int frame_id;
    int recv_result;

    // send to the socket
    if (send(socket_client, formed_message, strlen(formed_message), 0) < 0) {
        fprintf(stderr, "Sent error...\n");
        return -1;
    }
    printf("Sent message. Waiting for ACK/NACK...\n");

    int reply_length;
    char reply[64];
    // receive from the socket
    if ((reply_length = recv(socket_client, reply, 64, 0)) < 0) {
        fprintf(stderr, "Timeout. Re-transmitting...\n");
        return -1;
    }
    //  else if (!(strcmp(reply,"ACK")==0 || strcmp(reply,"NACK")==0))
    //  {
    // printf("Message received:  %s\n",reply );
    //  	fprintf(stderr, "Error in ACK or NACK!! Re-transmitting... \n");
    //      return -1;
    //  }
    reply[reply_length] = '\0';
    printf("Reply received: %s\n", reply);
    if (strcmp(reply, "NACK") == 0) {
        fprintf(stderr, "Previous transmission had some error. Re-transmitting...\n");
        return -1;
    } else if (strcmp(reply, "ACK") == 0) {
        return 1;
    } else {
        fprintf(stderr, "Error in ACK or NACK!! Re-transmitting... \n");
        return -1;
    }
}

char *
itoa(int value, char *result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) {
        *result = '\0';
        return result;
    }

    char *ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 +
                                                                                           (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}


int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "Usage ./client ip_addr port");
        return -1;
    }
    float BER;
    printf("Enter BER (probability of bit errors): ");
    scanf("%f", &BER);

    int socket_client;
    if ((socket_client = connection_setup(atoi(argv[2]), argv[1])) == -1) {
        return -1;
    }
    int i;
    unsigned long bit, crc;


    // at first, compute constant bit masks for whole CRC and CRC high bit

    crcmask = ((((unsigned long) 1 << (order - 1)) - 1) << 1) | 1;

    crchighbit = (unsigned long) 1 << (order - 1);

    generate_crc_table();

    crcinit_direct = crcinit;
    crc = crcinit;
    for (i = 0; i < order; i++) {

        bit = crc & 1;
        if (bit) crc ^= polynom;
        crc >>= 1;
        if (bit) crc |= crchighbit;
    }
    crcinit_nondirect = crc;

    while (true) {
        char message[1024];
        printf("Enter your message: ");
        scanf("%s", message);
        char rem_message[1024];
        char formed_message[10000];

        do {
            strcpy(formed_message, stringToBinary(message));
            unsigned long msg = crctablefast((unsigned char *) message, strlen(message));
            char *temp;
            temp = (char *) malloc(8192 * sizeof(char *));
            temp = itoa(msg, temp, 2);

            // printf("%s\n",temp);

            i = 0;
            char *e;
            e = (char *) malloc(8200 * sizeof(char *));
            if (strlen(temp) < 8) {
                for (i = 0; i < 8 - strlen(temp); i++) {
                    strcat(e, "0");
                }
                strcat(e, temp);
                strcpy(rem_message, e);
            }
            else {
                strcpy(rem_message, temp);
            }
            free(temp);
            free(e);

            strcat(formed_message, rem_message);
            //add error - flip bits randomly
            random_flip(formed_message, BER);
        } while (server_communicate(socket_client, formed_message) == -1);
    }

    close(socket_client);
    return 0;
}