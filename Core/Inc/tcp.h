#ifndef INC_TCP_H_
#define INC_TCP_H_

#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include <stdbool.h>

#define TCP_NO_DATA 	0xFF

typedef enum {
	TELNET_OK, /**< Everything is normal with the telnet server. */
	TELNET_ERR_CONNECTED, /**< There was an error in connection with the telnet server. */
	/*TELNET_ERR_BADALOC,*//**< There was an error allocating memory for the telnet server. */
	TELNET_ERR_BIND, /**< There was an error binding a socket to a port for the telnet server. */
	TELNET_ERR_PCBCREATE /**< There was an error creating a PCB structure for the telnet server. */
} TelnetStatus_t;

bool tcpIsConnected(void);
err_t TelnetPoll(void *arg, struct tcp_pcb *tpcb);
void TelnetProcessCharacter(char character);
char TelnetRead(void);
void TelnetWriteString(char* string);
void TelnetClose(void);
void TelnetWrite(const char character) ;
void TelnetRecvBufferWrite(uint8_t character);

TelnetStatus_t initTcpServer(void);

#define TELNET_BUFFER_LENGTH 		512
#define NOT_CONNECTED   			0
#define CONNECTED       			1

typedef struct {

	volatile unsigned long outstanding; 			/**< A count of the number of bytes that have been transmitted but have not yet been ACKed. */
	unsigned long close; 							/**< A value that is non-zero when the telnet connection should be closed down. */
	unsigned char buffer[TELNET_BUFFER_LENGTH]; 	/**< A buffer used to construct a packet of data to be transmitted to the telnet client. */
	volatile unsigned long length; 					/**< The number of bytes of valid data in the telnet packet buffer. */
	volatile unsigned long recvWrite; 				/**< The offset into g_pucTelnetRecvBuffer of the next location to be written in the buffer. The buffer is full if this value is one less than g_ulTelnetRecvRead (modulo the buffer size).*/
	struct tcp_pcb* pcb; 							/**< A pointer to the telnet session PCB data structure. */
} TelnetServer_t;


#ifndef TCP_RX_BUFFER_SIZE
  #define TCP_RX_BUFFER_SIZE 1024
#endif

#define TCP_RX_RING_BUFFER (TCP_RX_BUFFER_SIZE+1)

void tcpWrite(void *arg,uint16_t len);
#endif /* INC_TCP_ECHO_H_ */
