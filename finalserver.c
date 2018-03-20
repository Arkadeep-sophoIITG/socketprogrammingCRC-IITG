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
#include <signal.h>
#include <zconf.h>


#define FAIL -1
#define SUCCESS 1
#define ACK "ACK"
#define NACK "NACK"

// global so we can close them from the signal handler
int server_socket, client_socket;


typedef struct packet{
    char data[1024];
}Packet;

typedef struct frame{
    int frame_kind; //ACK:0, SEQ:1 FIN:2
    int sq_no;
    int ack;
    Packet packet;
}Frame;

void error(char *msg)
{
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


unsigned long reflect (unsigned long crc, int bitnum) {

    // reflects the lower 'bitnum' bits of 'crc'

    unsigned long i, j=1, crcout=0;

    for (i=(unsigned long)1<<(bitnum-1); i; i>>=1) {
        if (crc & i) crcout|=j;
        j<<= 1;
    }
    return (crcout);
}




void generate_crc_table() {

    // make CRC lookup table used by table algorithms

    int i, j;
    unsigned long bit, crc;

    for (i=0; i<256; i++) {

        crc=(unsigned long)i;
        if (refin) crc=reflect(crc, 8);
        crc<<= order-8;

        for (j=0; j<8; j++) {

            bit = crc & crchighbit;
            crc<<= 1;
            if (bit) crc^= polynom;
        }           

        if (refin) crc = reflect(crc, order);
        crc&= crcmask;
        crctab[i]= crc;
    }
}

unsigned long crctablefast (unsigned char* p, unsigned long len) {

    // fast lookup table algorithm without augmented zero bytes, e.g. used in pkzip.
    // only usable with polynom orders of 8, 16, 24 or 32.

    unsigned long crc = crcinit_direct;

    if (refin) crc = reflect(crc, order);

    if (!refin) while (len--) crc = (crc << 8) ^ crctab[ ((crc >> (order-8)) & 0xff) ^ *p++];
    else while (len--) crc = (crc >> 8) ^ crctab[ (crc & 0xff) ^ *p++];

    if (refout^refin) crc = reflect(crc, order);
    crc^= crcxor;
    crc&= crcmask;

    return(crc);
}

char *stringToBinary(char *s)
{
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
  if(binary == NULL){
     fprintf(stderr,"malloc has -1ed in stringToBinary(%s): %s\n",s, strerror(errno));
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






/* Iterative Function to calculate (x^y) in O(logy) */

float exponent(float x, int y)
{
    int res = 1;     // Initialize result
 
    while (y > 0)
    {
        // If y is odd, multiply x with result
        if (y & 1)
            res = res*x;
 
        // n must be even now
        y = y>>1; // y = y/2
        x = x*x;  // Change x to x^2
    }
    return res;
}

void random_flip(char *rem_message, float BER)
{
    unsigned int i =0;
    srand(time(0));
    for (i=0;i<strlen(rem_message);i++){
            if ((float)rand() / RAND_MAX < BER)
            {
                if (rem_message[i] == '0')
                {
                    rem_message[i] = '1';
                }
                else if (rem_message[i] == '1')
                {
                    rem_message[i] = '0';
                }
                else
                {
                    fprintf(stderr, "Convert to binary was not successful\n");
                }
            }
        }
}


char *construct_message(char *message,char *temp,int data_len)
{

    unsigned int i;
    unsigned int x=0;
    // convert the data from bits to char
    for (i = 0; i < data_len; i++)
    {
        int j;
        int v = 0;
        for (j = i; j < i + 8; j++)
        {
            if (message[j] == '1')
            {
                v += exponent(2, i + 7 - j);
            }
        }
        temp[x++]=(char)v;
        i = i + 7;
    }
    temp[x]='\0';
    return temp;
}

void process(int client_socket, float drop_probability,float ber)
{
    int data_len;
    char *temp_ack,*temp_nack,*final_ack,*final_nack;

    do
    {
        char message[10000];
        // receive from the socket
        data_len = read(client_socket, message, 10000);
        message[data_len] = '\0';
        
        // printf("%s\n",message);

        char *temp = (char *)malloc(10000*sizeof(char));
        memset(temp,0x00,10000);
        temp = construct_message(message,temp,data_len);
        
        unsigned long msg = crctablefast((unsigned char *)temp, strlen(temp));
        
        // printf("%lu\n",msg);
        memset(temp,0x00,10000);
        temp = construct_message(message,temp,data_len-8);
        // if crc is check is true, we have the correct data and we send ack else we send nack
        if (data_len && msg ==0)
        {
        	printf("Message received:  %s\n",temp);
            printf("Sending ACK....");
            srand(time(0));
            if (((float)rand() / RAND_MAX) < drop_probability)
            {
                printf("Packet dropped!\n");
                continue;
            }
            temp_ack =(char *)malloc(64*sizeof(char));
            memset(temp_ack,0x00,64);
            temp_ack = stringToBinary(ACK);
            random_flip(temp_ack,ber);
            final_ack=(char *)malloc(64*sizeof(char));
            memset(final_ack,0x00,64); 
            final_ack = construct_message(temp_ack,final_ack,strlen(temp_ack));
            // send to the socket
            int sent = send(client_socket, final_ack, strlen(final_ack), 0);
            printf("Sent successfully!\n");
            free(temp_ack);
            free(final_ack);
        }
        else if (data_len)
        {
           	printf("Message received:  %s\n",temp);
            printf("Message retrieved had some errors, sending NACK....");
            srand(time(0));
            if (((float)rand() / RAND_MAX) < drop_probability)
            {
                printf("Packet dropped!\n");
                continue;
            }

            temp_nack =(char *)malloc(64*sizeof(char));
            memset(temp_nack,0x00,64);
            temp_nack = stringToBinary(NACK);
            random_flip(temp_nack,ber);
            final_nack=(char *)malloc(64*sizeof(char));
            memset(final_nack,0x00,64); 
            final_nack = construct_message(temp_nack,final_nack,strlen(temp_nack));
            // send to the socket
            int sent = send(client_socket, final_nack, strlen(final_nack), 0);
            printf("Sent successfully!\n");
            free(temp_nack);
            free(final_nack);
        }
        else
        {
            close(client_socket);
        }
        free(temp);
    } while (data_len); // continue till we get proper data from the socket
}

int setup_connection(int port)
{
    // create a new socket witch AF_INET for internet domain, stream socket option, TCP(given by os) - reliable, connection oriented
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket: ");
        return -1;
    }
    printf("Created socket...\n");

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(port); // convert a port number in host byte order to a port number in network byte order
    server.sin_addr.s_addr = INADDR_ANY;
    bzero(&server.sin_zero, 8); // clears the buffer

    // bind function binds the socket to the address and port number specified in addr
    if ((bind(server_socket, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) < 0))
    {
        perror("binding: ");
        return -1;
    }
    printf("Done with binding...\n");

    // put the server socket in a passive mode, where it waits for the client to approach the server to make a connection
    // 5 defines the maximum length to which the queue of pending connections for sockfd may grow
    // if a connection request arrives when the queue is full, the client may receive an error
    if ((listen(server_socket, 5)) < 0)
    {
        perror("listening: ");
        return -1;
    }
    printf("Listening...\n");
    return 1;
}

void signal_callback(int signum)
{
    printf("Caught signal %d. Releasing resources...\n", signum);
    close(client_socket);
    close(server_socket);
    exit(signum);
}




int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: ./server port\n");
        return EXIT_FAILURE;
    }

    // handle the interrup signal like ctrl + c
    signal(SIGINT, signal_callback);
    float drop_probability;
    printf("Enter probability to drop packets: ");
    scanf("%f", &drop_probability);
    float BER;
    printf("Enter BER (probability of bit errors): ");
    scanf("%f", &BER);

    if (setup_connection(atoi(argv[1])) == -1)
    {
        return EXIT_FAILURE;
    }
      // at first, compute constant bit masks for whole CRC and CRC high bit
    int i;
    unsigned long bit, crc;

    crcmask = ((((unsigned long)1<<(order-1))-1)<<1)|1;
    crchighbit = (unsigned long)1<<(order-1);

    generate_crc_table();

    if (!direct) {

        crcinit_nondirect = crcinit;
        crc = crcinit;
        for (i=0; i<order; i++) {

            bit = crc & crchighbit;
            crc<<= 1;
            if (bit) crc^= polynom;
        }
        crc&= crcmask;
        crcinit_direct = crc;
    }

    else {

        crcinit_direct = crcinit;
        crc = crcinit;
        for (i=0; i<order; i++) {

            bit = crc & 1;
            if (bit) crc^= polynom;
            crc >>= 1;
            if (bit) crc|= crchighbit;
        }   
        crcinit_nondirect = crc;
    }

    while (1)
    {
        struct sockaddr_in client;
        unsigned int len;

        // extract the first connection request on the queue of pending connections for the listening socket
        // creates a new connected socket, and returns a new file descriptor referring to that socket
        // connection is established between client and server, and they are ready to transfer data
        if ((client_socket = accept(server_socket, (struct sockaddr *)&client, &len)) < 0)
        {
            error("Accepting connection ...");
        }
        printf("Accepted...\n");

        int pid;
        if ((pid = fork()) < 0)
        {
            error("Forking...");
        }
        if (pid == 0)
        {
            // child process closes the old socket and works with the new one
            close(server_socket);
            process(client_socket, drop_probability,BER);
            printf("Client served... Server can be terminated now!\n");

            // after working with the new socket, it simply exits
            return EXIT_SUCCESS;
        }
        else
        {
            // parent process does not need new socket, so it closes it
            // and keeps listening on old socket
            close(client_socket);
        }
    }
}
