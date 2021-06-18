#include "tcp.h"
#include <lwip/sockets.h>
#include "cmsis_os.h"

uint8_t tcp_rx_buffer[TCP_RX_RING_BUFFER];
volatile uint16_t tcp_rx_buffer_head = 0;
volatile uint16_t tcp_rx_buffer_tail = 0;

static struct tcp_pcb *telnet_pcb;
static TelnetServer_t telnet_server;
static bool IsConnected = false;
/*--------------------------------------------------------------------------------------------------------*/
/* PRIVATE FUNCTION PROTOTYPES */
/*--------------------------------------------------------------------------------------------------------*/
static err_t TelnetAccept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t TelnetReceive(void *arg, struct tcp_pcb *pcb, struct pbuf *p,err_t err);
//static err_t TelnetSent(void *arg, struct tcp_pcb *pcb, u16_t len);
static void TelnetError(void *arg, err_t err);

void tcp_reset_read_buffer(){
	tcp_rx_buffer_head = tcp_rx_buffer_tail;
}
#include "lwip/api.h"

err_t TelnetSent(void *arg, struct tcp_pcb *pcb, u16_t len) {
	/* See if this is for the game connection or for a secondary connection. */
	if (arg) {
		TelnetServer_t* server = (TelnetServer_t*) arg;
		/* Decrement the count of outstanding bytes. */
		server->outstanding -= len;
	}
	/* Return OK. */
	return (ERR_OK);
}

TelnetStatus_t initTcpServer(void) {
/**
	struct netconn *conn, *newconn;
	err_t err;
	// create a connection structure
	conn = netconn_new(NETCONN_TCP);
	// bind the connection to port 2000 on any local IP address
	netconn_bind(conn, NULL, 7);
	printf("Now listening\n");
	// tell the connection to listen for incoming connection requests
	netconn_listen(conn);
	// Grab new connection.
	err = netconn_accept(conn, &newconn);
	printf("accepted new connection %p\n", newconn);
	// Process the new connection.
	if (err == ERR_OK){
		struct netbuf *buf;void *data;u16_t len;

		while ((err = netconn_recv(newconn, &buf)) == ERR_OK){
			printf("Received data\n");
			do{
				netbuf_data(buf, &data, &len);
				err = netconn_write(newconn, data, len, NETCONN_COPY);
				if (err != ERR_OK) printf("tcpecho: netconn_write: error \"%s\"\n", lwip_strerr(err));
			}while (netbuf_next(buf) >= 0);
			netbuf_delete(buf);
		}

		// Close connection and discard connection identifier.
		netconn_close(newconn);
		netconn_delete(newconn);
	}
**/

	telnet_server.length = 0;
	telnet_server.outstanding = 0;
	for (uint_fast16_t i = 0; i < sizeof(telnet_server.buffer); ++i) {
		telnet_server.buffer[i] = 0;
	}
	//
	tcp_reset_read_buffer();
	/* Create a new tcp pcb */
	telnet_pcb = tcp_new();

	if (telnet_pcb != NULL) {
		err_t err;
		/* Bind telnet to port TELNET_PORT */
		err = tcp_bind(telnet_pcb, IP_ADDR_ANY, 1111);

		telnet_pcb->flags |= TF_NODELAY;
		telnet_pcb->flags |= TF_ACK_NOW;

		if (err == ERR_OK) {
			/* Start tcp listening for Telnet PCB */
			telnet_pcb = tcp_listen(telnet_pcb);
			/* Initialize LwIP tcp_accept callback function */
			tcp_accept(telnet_pcb, TelnetAccept);

			return TELNET_OK;
		} else {
			/* Deallocate the pcb */
			memp_free(MEMP_TCP_PCB, telnet_pcb);

			return TELNET_ERR_BIND;
		}
	} else {
		return TELNET_ERR_PCBCREATE;
	}
}

static err_t TelnetAccept(void *arg, struct tcp_pcb *pcb, err_t err) {

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	/* Check if already connected. */
	if (tcpIsConnected() == true) {
		/* There is already a connected client, so refuse this connection with
		 a message indicating this fact. */
		tcp_accepted(pcb);
		tcp_arg(pcb, NULL);
		tcp_sent(pcb, TelnetSent);
		//tcp_sent(pcb, NULL);
		tcp_output(pcb);
		/* Temporarily accept this connection until the message is transmitted. */
		return (ERR_OK);
	}

#undef TCP_PRIO_MAX
#define TCP_PRIO_MAX    250
	/* Setup the TCP connection priority. */
	tcp_setprio(pcb, TCP_PRIO_MAX );//TCP_PRIO_NORMAL

	/* Allocate structure server to maintain Telnet connection information */
	IsConnected = false;
	telnet_server.recvWrite = 0;
	telnet_server.length = 0;

	tcp_nagle_disable(pcb);

	telnet_server.pcb = pcb;
	/* Mark that a client has connected. */
	IsConnected = true;
	/* Accept this connection. */
	tcp_accepted(pcb);
	/* Setup the TCP callback argument. */
	tcp_arg(pcb, &telnet_server);
	/* Initialize lwIP tcp_recv callback function for pcb  */
	tcp_recv(pcb, TelnetReceive);
	/* Initialize lwIP tcp_err callback function for pcb  */
	tcp_err(pcb, TelnetError);
	/* Initialize lwIP tcp_poll callback function for pcb */
	//tcp_poll(pcb, TelnetPoll, 1);
	tcp_poll(pcb, NULL, 0);
	/* Setup the TCP sent callback function. */
	//--tcp_sent(pcb, NULL);
	tcp_sent(pcb, TelnetSent);
	/* Do not close the telnet connection until requested. */
	telnet_server.close = 0;
	/* Return a success code. */
	return ERR_OK;
}

void tcpWrite(void *arg,uint16_t len){
	if(tcpIsConnected()){
		taskENTER_CRITICAL();
		tcp_write(telnet_server.pcb, arg, len, TCP_WRITE_FLAG_COPY);
		tcp_output(telnet_server.pcb);
		taskEXIT_CRITICAL();
	}
}
//
static err_t TelnetReceive(void *arg, struct tcp_pcb *pcb, struct pbuf *p,err_t err) {
	struct pbuf *q;
	unsigned long ulIdx;
	unsigned char *pucData;
	char *recv;
	uint16_t next_head;

	TelnetServer_t* server;
	if (arg != NULL) {
		server = (TelnetServer_t*) arg;

		/* Process the incoming packet. */
		if ((err == ERR_OK) && (p != NULL)) {
			/* Accept the packet from TCP. */
			tcp_recved(pcb, p->tot_len);
		    //pointer to the pay load
			recv=(char *)p->payload;
			//
			switch(recv[0]){
				default :

				break ;
			}
			//
        	/* Loop through the pbufs in this packet. */
			for (q = p, pucData = (unsigned char*) q->payload; q != NULL;q = q->next) {
				/* Loop through the bytes in this pbuf. */
				for (ulIdx = 0; ulIdx < q->len; ulIdx++) {

					if (pucData[ulIdx] < 0x7F){
						/****** Process this character. ******/
						next_head = tcp_rx_buffer_head + 1;
						if (next_head == TCP_RX_RING_BUFFER) { next_head = 0; }
						// Write data to buffer unless it is full.
						if (next_head != tcp_rx_buffer_tail) {
							tcp_rx_buffer[tcp_rx_buffer_head] = pucData[ulIdx];
							tcp_rx_buffer_head = next_head;
						}
				    }

				}
			}
		    //
			/* Free the pbuf. */
			pbuf_free(p);
		} else if ((err == ERR_OK) && (p == NULL)) {
			/* If a null packet is passed in, close the connection. */
			server->length = 0;
			TelnetClose();
		}
	}
	/* Return okay. */
	return (ERR_OK);
}


static void TelnetError(void *arg, err_t err) {
	LWIP_UNUSED_ARG(err);
	TelnetServer_t* server;
	server = (TelnetServer_t*) arg;
	if (server != NULL) {
		printf("TCP err \r\n");
		/* free es structure */
		mem_free(server);
	}
	//
	IsConnected = false;
	initTcpServer();
}

err_t TelnetPoll(void *arg, struct tcp_pcb *tpcb) {
	err_t ret_err;
	TelnetServer_t* server;
	server = (TelnetServer_t*) arg;

	if (server != NULL) {
		unsigned long length = server->length;
		if ((server->pcb != NULL) && (length != 0)) {
			/* Write the data from the transmit buffer. */
			tcp_write(server->pcb, server->buffer, length, TCP_WRITE_FLAG_COPY);
			/* Increment the count of outstanding bytes. */
			server->outstanding += length;
			/* Output the telnet data. */
			tcp_output(server->pcb);
			/* Reset the size of the data in the transmit buffer. */
			server->length = 0;
		}
		/* See if the telnet connection should be closed; this will only occur once
		 all transmitted data has been ACKed by the client (so that some or all
		 of the final message is not lost). */
		if (server->pcb && (server->outstanding == 0) && (server->close != 0)) {
			TelnetClose();
		}
		ret_err = ERR_OK;
	} else {
		/* Nothing to be done */
		tcp_abort(tpcb);
		ret_err = ERR_ABRT;
	}
	return ret_err;
}


char TelnetRead(void) {
	uint16_t tail = tcp_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	if (tcp_rx_buffer_head == tail) {
	    return TCP_NO_DATA;
	} else {
	    uint8_t data = tcp_rx_buffer[tail];

	    tail++;
	    if (tail == TCP_RX_RING_BUFFER) { tail = 0; }
	    tcp_rx_buffer_tail = tail;

	    return data;
	}
}

void TelnetWrite(const char character) {
	/* Write this character into the output buffer. */
	telnet_server.buffer[telnet_server.length++] = character;
}

void TelnetWriteString(char* string) {
	if (tcpIsConnected() == true) {
		while (*string) {
			TelnetWrite(*string);
			++string;
		}
	}
}
//
bool tcpIsConnected(void) {
	return (IsConnected);
}
//
void TelnetClose(void) {

	struct tcp_pcb *pcb = telnet_server.pcb;

	/* Remove all callbacks */
	tcp_arg(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_err(pcb, NULL);
	tcp_poll(pcb, NULL, 0);
	/* Clear the telnet data structure pointer, to indicate that there is no longer a connection. */
	telnet_server.pcb = 0;
	/* Close tcp connection */
	tcp_close(pcb);
	IsConnected = false;
	/* Re-initialize the Telnet Server */
	initTcpServer();
}
